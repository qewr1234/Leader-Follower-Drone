#!/usr/bin/env python3
"""폐루프 특성화 테스트 — 실제 main.main()을 FC·카메라 없이 결정론적으로 비행시킨다.

sitl/harness.py 와 같은 자리를 스텁한다(cv2 / RealSense / YOLO). 다른 점은 FC까지 가짜라는 것:
가짜 Pixhawk가 BODY_NED 속도 setpoint를 받아 위치·기수를 적분하고, HEARTBEAT / LOCAL_POSITION_NED /
ATTITUDE / GLOBAL_POSITION_INT 를 돌려준다. 시계도 가짜라(프레임당 1/30초) 실행할 때마다 **같은
출력**이 나온다. 그래서 리팩토링 전후를 바이트 단위로 비교할 수 있다.

    python3 test_closed_loop.py                    # 시나리오 판정 (exit 0/1)
    python3 test_closed_loop.py --dump before.json # setpoint·상태 스트림 저장
    python3 test_closed_loop.py --compare before.json   # 저장본과 첫 차이점 보고

스텁은 인지 계층과 MAVLink 전송 계층뿐이다. 미션 상태머신 · IMM-EKF · 스케줄러 · 제어 · setpoint 생성은
저장소 코드가 그대로 돈다. mavlink_io.drain_messages 도 진짜다 (가짜 메시지 객체를 먹는다).

시나리오 (40초, 1200프레임):
   0~1s   ALT_HOLD 로 15m 호버, 리더 4.5m 전방
   1s     조종사가 GUIDED 로 넘김           → 미션 리셋, READY_HOVER
   3~11s  리더 0.3 m/s 전진               → FOLLOW, 거리 TARGET+v/KP 로 수렴
   11~16s 리더 정지                        → LEADER_HOVER, 거리 TARGET 로 수렴
   16~20s 리더 4초 소실(검출 없음)         → 2초 코스팅 뒤 LOST_HOLD
   20s    재검출, 리더는 호버 중           → 즉시 LEADER_HOVER (출발 확인 없이)
   25s~   리더 영구 소실                    → 10초 뒤(35s) LAND
"""

import argparse
import json
import math
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

_P = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
_P.add_argument("--dump", help="setpoint/상태 스트림을 JSON 으로 저장")
_P.add_argument("--compare", help="저장된 JSON 과 비교해 첫 차이를 보고")
_P.add_argument("--frames", type=int, default=1200)
ARGS = _P.parse_args()

import numpy as np  # noqa: E402

# ----------------------------------------------------------------- 스텁
class _Cv2(types.ModuleType):
    FONT_HERSHEY_SIMPLEX = 0

    def __getattr__(self, name):
        return lambda *a, **k: 0


class _MavConsts:
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 2
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 4
    POSITION_TARGET_TYPEMASK_VX_IGNORE = 8
    POSITION_TARGET_TYPEMASK_VY_IGNORE = 16
    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 64
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 128
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048
    MAV_FRAME_BODY_NED = 8
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_TYPE_GCS = 6
    MAV_TYPE_QUADROTOR = 2
    MAV_CMD_NAV_LAND = 21
    MAV_DATA_STREAM_ALL = 0
    MAV_DATA_STREAM_POSITION = 6
    MAV_DATA_STREAM_EXTRA1 = 10
    MAV_DATA_STREAM_EXTENDED_STATUS = 2


sys.modules["cv2"] = _Cv2("cv2")
for _n in ("pyrealsense2", "serial"):
    sys.modules[_n] = types.ModuleType(_n)
_u = types.ModuleType("ultralytics"); _u.YOLO = object; sys.modules["ultralytics"] = _u
_mv = types.ModuleType("pymavlink.mavutil"); _mv.mavlink = _MavConsts
_mv.mode_string_v10 = lambda msg: msg.mode_name
_pm = types.ModuleType("pymavlink"); _pm.mavutil = _mv
sys.modules["pymavlink"] = _pm; sys.modules["pymavlink.mavutil"] = _mv


class FakeClock:
    """main / mavlink_io 의 time 모듈을 대신한다. get_frames() 마다 1/30초 전진."""
    DT = 1.0 / 30.0

    def __init__(self, t0=1_000_000.0):
        self.t = t0
        self.t0 = t0

    def time(self):
        return self.t

    def sleep(self, s):
        pass

    def strftime(self, *a, **k):
        return "sim"

    def tick(self):
        self.t += self.DT

    @property
    def sim(self):
        return self.t - self.t0


CLOCK = FakeClock()

W, H = 640, 480
FX = FY = 384.0
CX, CY = W / 2.0, H / 2.0


class World:
    """팔로워(NED, m/rad)와 리더의 월드 상태. 가짜 FC가 팔로워를 움직이고, 시나리오가 리더를 움직인다."""
    f_n = 0.0; f_e = 0.0; f_d = -15.0; f_yaw = 0.0
    l_n = 3.0; l_e = 0.0; l_d = -15.0
    visible = True
    frames = 0
    dist = []            # (sim_t, front) — 추종 거리 이력

    @classmethod
    def relative_fru(cls):
        dn, de, dd = cls.l_n - cls.f_n, cls.l_e - cls.f_e, cls.l_d - cls.f_d
        c, s = math.cos(cls.f_yaw), math.sin(cls.f_yaw)
        return dn * c + de * s, -dn * s + de * c, -dd


def _bbox_px():
    front, right, up = World.relative_fru()
    front = max(front, 0.2)
    u = int(CX + FX * (right / front))
    v = int(CY - FY * (up / front))
    half = max(18, int(0.35 * FX / front))
    return u, v, half, front


# ----------------------------------------------------------------- 가짜 FC
class _Msg:
    def __init__(self, mtype, **fields):
        self._mtype = mtype
        self.__dict__.update(fields)

    def get_type(self): return self._mtype
    def get_srcSystem(self): return 1
    def get_srcComponent(self): return 1


class FakeFC:
    """BODY_NED 속도 setpoint 를 적분하는 최소 ArduCopter.

    GUIDED 에서만 setpoint 를 받고, 3초(GUID_TIMEOUT) 넘게 안 오면 정지한다. 속도는 0.3초 1차 지연.
    LAND 는 0.5 m/s 로 하강, 지면에서 disarm. 모드 변경은 set_mode() 한 곳으로만 들어온다.
    """
    GUID_TIMEOUT = 3.0
    TAU = 0.3

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = self
        self.mode = "ALT_HOLD"
        self.armed = True
        self.v_body = np.zeros(4)         # vx vy vz yaw_rate (실제)
        self.sp = None                    # 마지막 setpoint
        self.sp_time = -1e9
        self.setpoints = []               # (frame, sim_t, vx, vy, vz, yr)
        self.mode_calls = []              # (frame, sim_t, mode)
        self._pending = []

    # --- main / mavlink_io 가 부르는 것
    def set_position_target_local_ned_send(self, tbm, sysid, compid, frame, mask, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate):
        self.sp = np.array([vx, vy, vz, yaw_rate], dtype=float)
        self.sp_time = CLOCK.sim
        self.setpoints.append((World.frames, round(CLOCK.sim, 4), vx, vy, vz, yaw_rate))

    def command_long_send(self, *a): pass
    def request_data_stream_send(self, *a): pass
    def wait_heartbeat(self, *a, **k): return None

    def mode_mapping(self):
        return {"ALT_HOLD": 2, "GUIDED": 4, "LOITER": 5, "LAND": 9}

    def set_mode(self, name):
        self.mode_calls.append((World.frames, round(CLOCK.sim, 4), name))
        self.mode = name

    def recv_match(self, blocking=False, **k):
        return self._pending.pop(0) if self._pending else None

    # --- 시뮬레이션
    def step(self, dt):
        if self.mode == "GUIDED" and self.armed and self.sp is not None and CLOCK.sim - self.sp_time <= self.GUID_TIMEOUT:
            v_cmd = self.sp
        elif self.mode == "LAND" and self.armed:
            v_cmd = np.array([0.0, 0.0, 0.5, 0.0])
        else:
            v_cmd = np.zeros(4)
        self.v_body += (v_cmd - self.v_body) * (dt / self.TAU)
        vx, vy, vz, yr = self.v_body
        c, s = math.cos(World.f_yaw), math.sin(World.f_yaw)
        World.f_n += (vx * c - vy * s) * dt
        World.f_e += (vx * s + vy * c) * dt
        World.f_d += vz * dt
        World.f_yaw = math.atan2(math.sin(World.f_yaw + yr * dt), math.cos(World.f_yaw + yr * dt))
        if self.mode == "LAND" and World.f_d >= 0.0:
            World.f_d = 0.0
            self.armed = False

        base_mode = 128 if self.armed else 0
        self._pending = [
            _Msg("HEARTBEAT", type=2, base_mode=base_mode, mode_name=self.mode),
            _Msg("LOCAL_POSITION_NED", x=World.f_n, y=World.f_e, z=World.f_d,
                 vx=vx * c - vy * s, vy=vx * s + vy * c, vz=vz),
            _Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=World.f_yaw, rollspeed=0.0, pitchspeed=0.0, yawspeed=yr),
            _Msg("GLOBAL_POSITION_INT", lat=358300000, lon=1287500000, alt=int((50 - World.f_d) * 1000),
                 relative_alt=int(-World.f_d * 1000), vx=0, vy=0, vz=0, hdg=int(math.degrees(World.f_yaw) % 360 * 100)),
        ]


FC = FakeFC()


# ----------------------------------------------------------------- 시나리오
def scenario(t):
    """sim 시간 t 에서 리더/조종사가 하는 일. 프레임마다 get_frames() 에서 호출."""
    if 1.0 <= t < 1.0 + CLOCK.DT and FC.mode == "ALT_HOLD":
        FC.set_mode("GUIDED")                       # 조종사가 스위치를 넘김
    if 3.0 <= t < 11.0:
        World.l_n += 0.3 * CLOCK.DT                 # 리더 전진
    World.visible = not (16.0 <= t < 20.0) and t < 25.0


# ----------------------------------------------------------------- 인지 스텁
class FakeCam:
    def __init__(self, *a, **k):
        self.depth_scale = 0.001
        self.intrinsics = {"fx": FX, "fy": FY, "ppx": CX, "ppy": CY}

    def start(self): pass
    def stop(self): pass

    def get_frames(self):
        if World.frames >= ARGS.frames:
            raise KeyboardInterrupt("scenario finished")
        World.frames += 1
        CLOCK.tick()
        FC.step(CLOCK.DT)
        scenario(CLOCK.sim)
        World.dist.append((CLOCK.sim, World.relative_fru()[0]))
        color = np.zeros((H, W, 3), dtype=np.uint8)
        depth = np.full((H, W), 15000, dtype=np.uint16)
        if World.visible:
            u, v, half, front = _bbox_px()
            if 0 <= u < W and 0 <= v < H:
                depth[max(0, v - half):v + half, max(0, u - half):u + half] = int(front / self.depth_scale)
        return color, depth


class FakeDetector:
    def __init__(self, *a, **k): pass

    def detect(self, image, roi=None):
        if not World.visible:
            return []
        u, v, half, _ = _bbox_px()
        if not (0 <= u < W and 0 <= v < H):
            return []
        return [{"bbox": [float(u - half), float(v - half), float(u + half), float(v + half)],
                 "conf": 0.85, "cls_name": "leader_drone"}]


# ----------------------------------------------------------------- main 연결
import camera, detector  # noqa: E402,E401
camera.D435i = FakeCam
detector.YoloDetector = FakeDetector
detector.load_model = lambda *a, **k: None

import main, mavlink_io, mission_manager  # noqa: E402,E401

main.time = CLOCK
mavlink_io.time = CLOCK
main.D435i = FakeCam
main.YoloDetector = FakeDetector
main.SHOW_WINDOW = False
main.USE_LEADER_ESP32 = False
main.SEND_MAVLINK_COMMANDS = True
main.CONFIG["logger"]["enabled"] = False
main.connect_fc = lambda: FC

STATES = []          # (frame, sim_t, state)


class RecordingMission(mission_manager.MissionManager):
    def update(self, *a, **k):
        st, pol = super().update(*a, **k)
        if not STATES or STATES[-1][2] != st:
            STATES.append((World.frames, round(CLOCK.sim, 4), st))
        return st, pol


main.MissionManager = RecordingMission

# ----------------------------------------------------------------- 실행
import io, contextlib  # noqa: E402,E401
_buf = io.StringIO()
err = None
try:
    with contextlib.redirect_stdout(_buf):
        main.main()
except Exception as e:                       # main.main() 은 KeyboardInterrupt 를 스스로 잡는다
    import traceback
    err = f"{type(e).__name__}: {e}\n{traceback.format_exc()}"

failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        failures.append(name)


def state_at(t):
    cur = None
    for _, st_t, st in STATES:
        if st_t <= t:
            cur = st
    return cur


def first_time(state, after=0.0):
    for _, st_t, st in STATES:
        if st == state and st_t >= after:
            return st_t
    return None


check("main.main() 이 예외 없이 끝남", err is None, err or "")
print(f"프레임 {World.frames}, setpoint {len(FC.setpoints)}개, 모드 변경 {FC.mode_calls}")
print("상태 이력:", [f"{t:.1f}s:{s}" for _, t, s in STATES])

hz = len([s for s in FC.setpoints if 5.0 <= s[1] <= 15.0]) / 10.0
# 위상 고정 송신이라 프레임 간격(1/30)과 무관하게 평균 10Hz 여야 한다 (프레임 경계 정렬 방식이면 7.5Hz 였다).
check("setpoint 송신율 9.5~10.5Hz (위상 고정)", 9.5 <= hz <= 10.5, f"{hz:.1f}Hz")
check("GUIDED 진입 후 READY_HOVER", state_at(2.5) == "READY_HOVER", f"{state_at(2.5)}")
t_follow = first_time("FOLLOW")
check("리더 출발 후 FOLLOW 진입 (3s + 0.7s 확인 안팎)", t_follow is not None and 3.5 <= t_follow <= 5.5, f"t={t_follow}")

# 리더가 멈춘 뒤 팔로워가 따라붙는 동안은 상대속도가 남아 FOLLOW/LEADER_HOVER 를 오간다(히스테리시스,
# 둘 다 allow_follow). 소실 직전(16s)에는 정착해서 LEADER_HOVER 여야 한다.
check("리더 정지 후 소실 직전까지 정착 → LEADER_HOVER", state_at(15.9) == "LEADER_HOVER", f"{state_at(15.9)}")


def front_at(t):
    return min(World.dist, key=lambda d: abs(d[0] - t))[1]


_e105 = front_at(10.5) - main.TARGET_DISTANCE_M
_dmin = min(d for t, d in World.dist if 11.0 <= t <= 16.0)
# 리더는 3.0s 에 출발, FOLLOW 확정은 ~5.1s 라 그 사이 0.6m 가 벌어진 채 시작한다. 같은 조건에서 P 만이면 +1.15m.
check("FOLLOW 중(리더 0.3m/s, t=10.5s) 거리 오차 < 0.7m — 피드포워드 (KFF=0 이면 +1.15m, 정상상태 v/Kp=1.36m)",
      abs(_e105) < 0.7, f"front-target={_e105:+.2f}m")
check("리더 정지 후 최소 접근 거리 ≥ 2.3m (피드포워드 오버슈트 없음)", _dmin >= 2.3, f"min={_dmin:.2f}m")
check("추종 중(t=8s) 상태는 FOLLOW — 리더 절대 속도 기준 (상대 속도면 따라잡는 순간 LEADER_HOVER 로 오판)",
      state_at(8.0) == "FOLLOW", f"{state_at(8.0)}")
check("리더 정지(11s) 후 2.5s 안에 LEADER_HOVER", state_at(13.5) == "LEADER_HOVER", f"{state_at(13.5)}")
t_lost = first_time("LOST_HOLD", 16.0)
check("4초 소실 → 2초 코스팅 뒤 LOST_HOLD", t_lost is not None and 17.8 <= t_lost <= 18.6, f"t={t_lost}")
t_resume = None
for _, st_t, st in STATES:
    if st_t >= 20.0 and st in ("LEADER_HOVER", "FOLLOW"):
        t_resume = st_t
        break
check("재검출(리더 호버 중) 즉시 재개 — READY_HOVER 에 갇히지 않음", t_resume is not None and t_resume < 20.6, f"t={t_resume}")
check("재검출 후 READY_HOVER 로 떨어지지 않음", state_at(24.0) in ("LEADER_HOVER", "FOLLOW"), f"{state_at(24.0)}")

land = [c for c in FC.mode_calls if c[2] == "LAND"]
check("영구 소실 25s → 10초 뒤 LAND 1회", len(land) == 1 and 34.0 <= land[0][1] <= 36.5, f"{land}")
check("LAND 이후 FC 가 하강 중 (가짜 FC 가 LAND 를 받아들임)", land and World.f_d > -15.0 + 1.0, f"d={World.f_d:.2f}")

stream = {
    "setpoints": [[f, t, round(vx, 9), round(vy, 9), round(vz, 9), round(yr, 9)] for f, t, vx, vy, vz, yr in FC.setpoints],
    "mode_calls": [list(c) for c in FC.mode_calls],
    "states": [list(s) for s in STATES],
    "final_pose": [round(World.f_n, 6), round(World.f_e, 6), round(World.f_d, 6), round(World.f_yaw, 6)],
}

if ARGS.dump:
    Path(ARGS.dump).write_text(json.dumps(stream))
    print(f"스트림 저장: {ARGS.dump}")

if ARGS.compare:
    ref = json.loads(Path(ARGS.compare).read_text())
    same = True
    for key in ("mode_calls", "states", "final_pose"):
        if ref[key] != stream[key]:
            same = False
            print(f"[DIFF] {key}:\n   ref={ref[key]}\n   now={stream[key]}")
    n = min(len(ref["setpoints"]), len(stream["setpoints"]))
    for i in range(n):
        a, b = ref["setpoints"][i], stream["setpoints"][i]
        if a[:2] != b[:2] or any(abs(x - y) > 1e-9 for x, y in zip(a[2:], b[2:])):
            same = False
            print(f"[DIFF] setpoint #{i}: ref={a} now={b}")
            break
    if len(ref["setpoints"]) != len(stream["setpoints"]):
        same = False
        print(f"[DIFF] setpoint 개수 ref={len(ref['setpoints'])} now={len(stream['setpoints'])}")
    check(f"저장본({ARGS.compare})과 스트림 동일", same)

print()
print(f"{'FAILED: ' + ', '.join(failures) if failures else '폐루프 검사 전부 통과'} ({len(failures)} 실패)")
sys.exit(1 if failures else 0)
