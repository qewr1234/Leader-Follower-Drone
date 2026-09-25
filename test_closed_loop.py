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
# 안전 시나리오 (docs/FLIGHT_SAFETY_CHECKLIST.md). default 만 --dump/--compare 골든 스트림의 대상이다.
# 모두 리더가 3 s 에 출발해 미션이 FOLLOW 에 들어간 뒤의 일이다(출발 확인 전에는 명령이 나가지 않는다).
#   tilt     : 리더 3~6 s 전진 후 정지, 8 s 부터 기체가 pitch −10° 로 기운 채(맞바람) 정지 리더를 본다 — 수평화(--level) 유무에 따른 vz 편향
#   nan      : 6 s 에 EKF 상태가 NaN 이 된다 — setpoint 가 +0.35 전진으로 둔갑하지 않고, 추정기가 리셋돼 추종이 재개된다
#   fc_stale : 리더 3~16 s 전진, 8~12 s FC 텔레메트리(HEARTBEAT 포함) 가 끊긴다 — 1 s 뒤 정지 명령, 3 s 뒤 모드 불명(LAND 금지), 복구 시 미션 리셋
#   climb    : 리더 3~5 s 전진 후 0.1 m/s 로 계속 상승(--frames 1900) — 인계 고도 + MAX_CLIMB_ABOVE_ENTRY_M 에서 상승 명령이 멈춘다
#   lost_alt : 25 s 영구 소실 뒤 30 s 부터 HEARTBEAT 만 끊긴다 — 35 s 의 FAILSAFE_LAND 에서 LAND 를 보내지 않는다 (--autonomous-land 1 로 실행)
#   takeover : HEARTBEAT 1 Hz, 25 s 영구 소실, 조종사가 34.6 s 에 LOITER 로 탈환 — 35.0 s 의 FAILSAFE_LAND 결정이 직전 heartbeat(34.03 s,
#              GUIDED) 를 근거로 LAND 를 보내 조종사를 덮어쓰면 안 된다 (--autonomous-land 1 로 실행)
#   sine     : 리더 0.25 ± 0.05 m/s, 1.15 rad/s 정현파 (SITL leader_sine 과 같은 자극) — analysis/sine_gain.py 검증용 로그 (--log-dir)
_P.add_argument("--scenario", default="default", choices=["default", "tilt", "nan", "fc_stale", "climb", "lost_alt", "takeover", "sine"])
_P.add_argument("--level", type=int, default=None, help="controller.level_by_attitude 강제 (0/1). 없으면 config 값")
_P.add_argument("--autonomous-land", type=int, default=None, help="mission.autonomous_land 강제 (0/1). 없으면 config 값(False)")
_P.add_argument("--log-dir", default=None, help="이 디렉터리에 main 의 JSONL 로그를 남긴다 (분석 스크립트 검증용). 가짜 UWB 거리 GT 도 켠다")
_P.add_argument("--core", default=None, choices=["py", "cpp"], help="추정 코어 (환경변수 MARS_CORE 와 같음). cpp 는 cpp/build.sh 로 만든 mars_core")
ARGS = _P.parse_args()
if ARGS.core:
    import os as _os
    _os.environ["MARS_CORE"] = ARGS.core

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

    def monotonic(self):
        return self.t

    def perf_counter(self):
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
    f_roll = 0.0; f_pitch = 0.0        # tilt 시나리오: 맞바람에 기운 자세 (이동 없이 자세만)
    l_n = 3.0; l_e = 0.0; l_d = -15.0
    visible = True
    alt_hist = []        # (sim_t, 팔로워 고도)
    frames = 0
    dist = []            # (sim_t, front) — 추종 거리 이력

    @classmethod
    def relative_fru(cls):
        dn, de, dd = cls.l_n - cls.f_n, cls.l_e - cls.f_e, cls.l_d - cls.f_d
        c, s = math.cos(cls.f_yaw), math.sin(cls.f_yaw)
        return dn * c + de * s, -dn * s + de * c, -dd


def _rot_body_to_ned(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _bbox_px():
    """리더를 기체 고정 카메라로 투영. roll/pitch 가 0 이면 예전 식(yaw 만) 과 같다."""
    if World.f_roll == 0.0 and World.f_pitch == 0.0:
        front, right, up = World.relative_fru()
    else:
        rel_ned = np.array([World.l_n - World.f_n, World.l_e - World.f_e, World.l_d - World.f_d])
        frd = _rot_body_to_ned(World.f_roll, World.f_pitch, World.f_yaw).T @ rel_ned
        front, right, up = float(frd[0]), float(frd[1]), float(-frd[2])
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
        self.mute = False                 # True 면 텔레메트리를 전혀 내지 않는다 (링크 정체)
        self.mute_heartbeat = False       # True 면 HEARTBEAT 만 내지 않는다
        self.hb_period = 0.0              # >0 이면 HEARTBEAT 를 이 주기로만 낸다 (ArduCopter 1 Hz 고정). 0 = 매 프레임(골든)
        self._last_hb = -1e9

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
        World.alt_hist.append((CLOCK.sim, -World.f_d))
        if self.mute:
            self._pending = []
            return
        self._pending = [
            _Msg("HEARTBEAT", type=2, base_mode=base_mode, mode_name=self.mode),
            _Msg("LOCAL_POSITION_NED", x=World.f_n, y=World.f_e, z=World.f_d,
                 vx=vx * c - vy * s, vy=vx * s + vy * c, vz=vz),
            _Msg("ATTITUDE", roll=World.f_roll, pitch=World.f_pitch, yaw=World.f_yaw, rollspeed=0.0, pitchspeed=0.0, yawspeed=yr),
            _Msg("GLOBAL_POSITION_INT", lat=358300000, lon=1287500000, alt=int((50 - World.f_d) * 1000),
                 relative_alt=int(-World.f_d * 1000), vx=0, vy=0, vz=0, hdg=int(math.degrees(World.f_yaw) % 360 * 100)),
        ]
        if self.mute_heartbeat:
            self._pending = [m for m in self._pending if m.get_type() != "HEARTBEAT"]
        elif self.hb_period > 0.0:
            if CLOCK.sim - self._last_hb + 1e-9 >= self.hb_period:
                self._last_hb = CLOCK.sim
            else:
                self._pending = [m for m in self._pending if m.get_type() != "HEARTBEAT"]


FC = FakeFC()


# ----------------------------------------------------------------- 시나리오
def scenario_default(t):
    """sim 시간 t 에서 리더/조종사가 하는 일. 프레임마다 get_frames() 에서 호출."""
    if 1.0 <= t < 1.0 + CLOCK.DT and FC.mode == "ALT_HOLD":
        FC.set_mode("GUIDED")                       # 조종사가 스위치를 넘김
    if 3.0 <= t < 11.0:
        World.l_n += 0.3 * CLOCK.DT                 # 리더 전진
    World.visible = not (16.0 <= t < 20.0) and t < 25.0


def _handover(t):
    if 1.0 <= t < 1.0 + CLOCK.DT and FC.mode == "ALT_HOLD":
        FC.set_mode("GUIDED")


def scenario_tilt(t):
    """리더 3~6 s 전진 후 정지(LEADER_HOVER, 명령 허용). 8 s 부터 맞바람으로 pitch −10°(기수 하향) — 이동 없이 자세만,
    FC 가 위치를 잡고 있는 상태의 자세 편향."""
    _handover(t)
    if 3.0 <= t < 6.0:
        World.l_n += 0.3 * CLOCK.DT
    World.f_pitch = math.radians(-10.0) if t >= 8.0 else 0.0


def scenario_nan(t):
    _handover(t)
    if 3.0 <= t < 11.0:
        World.l_n += 0.3 * CLOCK.DT


def scenario_fc_stale(t):
    _handover(t)
    if 3.0 <= t < 16.0:
        World.l_n += 0.3 * CLOCK.DT
    FC.mute = 8.0 <= t < 12.0


def scenario_climb(t):
    _handover(t)
    if 3.0 <= t < 5.0:
        World.l_n += 0.3 * CLOCK.DT
    if t >= 5.0:
        World.l_d -= 0.1 * CLOCK.DT                 # 리더 상승 0.1 m/s (NED 라 d 감소). 팔로워 MAX_VZ 0.12 라 따라갈 수 있다


def scenario_lost_alt(t):
    scenario_default(t)
    FC.mute_heartbeat = t >= 30.0


SINE_OMEGA, SINE_V0, SINE_AMP = 1.15, 0.25, 0.05


def scenario_sine(t):
    _handover(t)
    if t >= 3.0:
        World.l_n += (SINE_V0 + SINE_AMP * math.sin(SINE_OMEGA * (t - 3.0))) * CLOCK.DT


def scenario_takeover(t):
    FC.hb_period = 1.0
    scenario_default(t)
    if 34.6 <= t < 34.6 + CLOCK.DT and FC.mode == "GUIDED":
        FC.set_mode("LOITER")                       # 조종사 탈환 — 다음 heartbeat(35.03 s) 전
    World.visible = World.visible and t < 25.0


_SCENARIOS = {"default": scenario_default, "tilt": scenario_tilt, "nan": scenario_nan,
              "fc_stale": scenario_fc_stale, "climb": scenario_climb, "lost_alt": scenario_lost_alt,
              "takeover": scenario_takeover, "sine": scenario_sine}


def scenario(t):
    _SCENARIOS[ARGS.scenario](t)


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
        World.dist.append((CLOCK.sim, World.relative_fru()[0], World.visible))
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

import main, mavlink_io  # noqa: E402,E401

main.time = CLOCK
mavlink_io.time = CLOCK
main.D435i = FakeCam
main.YoloDetector = FakeDetector
main.SHOW_WINDOW = False
main.USE_LEADER_ESP32 = False
main.SEND_MAVLINK_COMMANDS = True
main.CONFIG["logger"]["enabled"] = False
main.connect_fc = lambda: FC
if ARGS.level is not None:
    main.LEVEL_BY_ATTITUDE = bool(ARGS.level)
if ARGS.autonomous_land is not None:
    main.AUTONOMOUS_LAND = bool(ARGS.autonomous_land)
if ARGS.log_dir:
    main.CONFIG["logger"]["enabled"] = True
    main.CONFIG["logger"]["log_dir"] = ARGS.log_dir
    # 가짜 UWB: 선두↔후미 실제 3D 거리 + 3 cm 잡음, 10 Hz. 앵커/태그 오프셋 0.
    import uwb_reader as _uwb
    _rng_uwb = np.random.default_rng(7)

    class _FakeUwbRx:
        def __init__(self):
            self.latest, self._last_t = None, -1e9

        def read_latest(self):
            if CLOCK.sim - self._last_t >= 0.1:
                self._last_t = CLOCK.sim
                d = math.sqrt((World.l_n - World.f_n) ** 2 + (World.l_e - World.f_e) ** 2 + (World.l_d - World.f_d) ** 2)
                self.latest = _uwb.UwbRange(range_m=d + float(_rng_uwb.normal(0.0, 0.03)), rx_time=CLOCK.t, seq=int(CLOCK.sim * 10))
            return self.latest

        def close(self):
            pass
    main.UWB_ENABLED, main.UWB_KIND = True, "serial"
    main.open_uwb_receiver = lambda: _FakeUwbRx()
if ARGS.scenario == "nan":
    _nan_fired = []

    class _NanEkf(main.ImmEkf):
        """6 s 에 상태 벡터를 NaN 으로 오염시킨다 (특이 S / 극단 dt 가 일으킬 수 있는 종류)."""
        def predict(self, dt):
            super().predict(dt)
            if not _nan_fired and CLOCK.sim >= 6.0 and self.initialized:
                _nan_fired.append(CLOCK.sim)
                if hasattr(self, "set_filter_x"):          # C++ 코어
                    for i in range(2):
                        self.set_filter_x(i, np.full(6, float("nan")))
                else:
                    for f in self.filters:
                        f.x[:] = float("nan")
                self.mark_dirty()
    main.ImmEkf = _NanEkf

STATES = []          # (frame, sim_t, state)


class RecordingMission(main.MissionManager):   # --core cpp 면 mars_core.MissionManager 를 감싼다
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


def _sp_between(t0, t1):
    return [s for s in FC.setpoints if t0 <= s[1] <= t1]


def _alt_at(t):
    return min(World.alt_hist, key=lambda a: abs(a[0] - t))[1]


if ARGS.scenario == "tilt":
    _vz = [s[4] for s in _sp_between(8.0, 12.0)]
    _dalt = _alt_at(39.0) - _alt_at(2.0)
    if main.LEVEL_BY_ATTITUDE:
        check("안전(tilt, 수평화 ON): pitch −10° 로 기운 채 같은 고도 정지 리더를 봐도 vz 명령 |vz| < 0.01 m/s, 고도 변화 < 0.05 m",
              _vz and max(abs(v) for v in _vz) < 0.01 and abs(_dalt) < 0.05, f"max|vz|={max(abs(v) for v in _vz) if _vz else float('nan'):.3f} Δalt={_dalt:+.3f} m")
    else:
        check("안전(tilt, 수평화 OFF — 결함 재현): pitch −10° 에 vz 명령 −0.05 m/s 이하(상승) 가 나가고 팔로워가 D·tan10° ≈ 0.5 m 위로 올라가 정착",
              _vz and min(_vz) < -0.05 and 0.3 < _dalt < 0.8, f"min vz={min(_vz) if _vz else float('nan'):+.3f} Δalt={_dalt:+.3f} m")
elif ARGS.scenario == "nan":
    _pre = _sp_between(5.7, 6.0)
    _post = _sp_between(6.0, 6.5)
    _vx_pre = _pre[-1][2] if _pre else float("nan")
    check("안전(nan): EKF 상태가 NaN 이 돼도 setpoint 는 전부 유한하고 한계 안이며, NaN 직후 0.5 s 의 전진 명령이 직전 값 +0.05 를 넘지 않는다(+0.35 로 둔갑 없음), 루프 생존",
          err is None and _post and all(math.isfinite(x) for s in FC.setpoints for x in s[2:]) and all(abs(s[2]) <= main.MAX_VX + 1e-9 for s in _post)
          and max(s[2] for s in _post) < _vx_pre + 0.05,
          f"vx 직전={_vx_pre:.3f} 직후={[round(s[2], 3) for s in _post]}")
    check("안전(nan): 추정기가 리셋돼 다음 측정에서 다시 시작하고 추종이 재개된다 (t=8~10 s 전진 명령 > 0.05)",
          _nan_fired and any(s[2] > 0.05 for s in _sp_between(8.0, 10.0)), f"nan@{_nan_fired} n={len(_sp_between(8.0, 10.0))}")
elif ARGS.scenario == "fc_stale":
    _hold = _sp_between(9.5, 12.0)
    _before = _sp_between(7.0, 8.0)
    check("안전(fc_stale): FC 텔레메트리가 8.0 s 에 끊기면 1 s 뒤부터 정지 명령이 나가 평활 감쇠 후 9.5 s 부터 |v| < 0.02 — 직전에는 추종 중(vx>0)",
          _before and any(s[2] > 0.05 for s in _before) and _hold and all(abs(s[2]) < 0.02 and abs(s[4]) < 0.02 for s in _hold),
          f"7~8s max vx={max(s[2] for s in _before) if _before else float('nan'):.2f}, 9.5~12s max|vx|={max(abs(s[2]) for s in _hold) if _hold else float('nan'):.3f}")
    check("안전(fc_stale): 3 s 이상 끊겨 모드 불명이 된 뒤 복구(12 s) 하면 GUIDED 진입과 같이 미션 리셋(WAIT_LEADER) → 출발 확인 뒤 FOLLOW 재개, LAND 없음",
          any(st_t >= 12.0 and st == "READY_HOVER" for _, st_t, st in STATES) and first_time("FOLLOW", 12.0) is not None and not FC.mode_calls[1:],
          f"states={[f'{t:.1f}:{s}' for _, t, s in STATES if t >= 11.0]} modes={FC.mode_calls}")
elif ARGS.scenario == "climb":
    _entry = _alt_at(1.1)
    _mid = _alt_at(45.0)
    _max = max(a for t, a in World.alt_hist if t <= 62.0)
    _end = _alt_at(62.0)
    check("안전(climb): 리더 0.1 m/s 상승을 따라 올라가다가(45 s 에 +3 m 이상) 인계 고도 + MAX_CLIMB_ABOVE_ENTRY_M(5 m) 천장에 붙어 멈춘다(최대 +5.3 m 이하, 62 s 에 +4.7~5.3 m)",
          _mid > _entry + 3.0 and _max <= _entry + main.MAX_CLIMB_ABOVE_ENTRY_M + 0.3 and abs(_end - _entry - main.MAX_CLIMB_ABOVE_ENTRY_M) <= 0.3,
          f"entry={_entry:.2f} 45s=+{_mid - _entry:.2f} max=+{_max - _entry:.2f} 62s=+{_end - _entry:.2f} m")
elif ARGS.scenario == "sine":
    _fv = [s[2] for s in _sp_between(15.0, 39.9)]
    check("정현파(sine): 리더 0.25±0.05 m/s·1.15 rad/s 를 추종하며 FOLLOW/LEADER_HOVER 에 머물고 전진 명령이 유한·한계 안",
          all(math.isfinite(v) and abs(v) <= main.MAX_VX + 1e-9 for v in _fv) and state_at(30.0) in ("FOLLOW", "LEADER_HOVER"),
          f"n={len(_fv)} state@30={state_at(30.0)}")
    if ARGS.log_dir:
        import glob
        from analysis import sine_gain as _sg, nees_nis as _nn, identify_plant as _ip
        from analysis.logtools import Log as _Log
        _path = sorted(glob.glob(str(Path(ARGS.log_dir) / "*.jsonl")))[-1]
        _lg = _Log.load(_path)
        _r = _sg.analyze(_lg, omega=SINE_OMEGA, t0=15.0, t1=40.0, leader="uwb")
        _pr = _sg.predicted_gain(SINE_OMEGA)
        check("정현파(sine) 로그 → analysis/sine_gain.py(uwb): 실측 |Γ| 이 선형 모델 예측의 0.9~1.1 배, 위상 ±10° (실제 코드 폐루프 ↔ 선형 모델 ↔ 분석 도구 3자 일치)",
              0.9 <= _r["gain"] / _pr["gain"] <= 1.1 and abs(_r["phase_deg"] - _pr["phase_deg"]) <= 10.0,
              f"측정 {_r['gain']:.3f}±{_r['gain_std']:.3f} @{_r['phase_deg']:+.0f}° 예측 {_pr['gain']:.3f} @{_pr['phase_deg']:+.0f}°")
        _cons = _nn.analyze(_lg, t0=5.0, uwb_sigma=0.03)
        check("정현파(sine) 로그 → analysis/nees_nis.py: RGB-D NIS n>500, 거리 NEES n>500 이고 잡음 없는 가짜 세계에서는 과소신뢰(NIS 평균 < 3, 잔차 σ ≤ 0.05 m) 로 판정",
              _cons["nis"]["rgbd"]["n"] > 500 and _cons["range_nees"]["n"] > 500 and _cons["nis"]["rgbd"]["mean"] < 3.0 and _cons["range_nees"]["resid_std_m"] <= 0.05,
              f"NIS n={_cons['nis']['rgbd']['n']} mean={_cons['nis']['rgbd']['mean']:.2f}; NEES n={_cons['range_nees']['n']} resid σ={_cons['range_nees']['resid_std_m']:.3f}")
        _t, _u, _y = _ip.load_main_log(_path)
        _id = _ip.fit_fopdt(_t, _u[:, 0], _y[:, 0])
        check("정현파(sine) 로그 → analysis/identify_plant.py: 가짜 FC(τ 0.3 s, 10 Hz ZOH) 를 K 0.95~1.05, τ+L 0.25~0.45 s, fit > 95 % 로 되찾음",
              0.95 <= _id["K"] <= 1.05 and 0.25 <= _id["t63_s"] <= 0.45 and _id["fit_pct"] > 95.0,
              f"K={_id['K']:.3f} tau={_id['tau']:.3f} L={_id['L']:.3f} fit={_id['fit_pct']:.1f}%")
        _last = _lg.rows[-1]
        check("정현파(sine) 로그: 논문 분석에 필요한 열이 전부 있다 (t_mono, uwb.range_center_m, ekf.P_pos, reliability.gate_d2, control.body_vx, vehicle_state.local_position.vx)",
              all(k in _last for k in ("t_mono", "uwb.range_center_m", "ekf.P_pos", "reliability.gate_d2", "control.body_vx", "vehicle_state.local_position.vx", "uwb.residual_m")),
              f"keys={len(_last)}")
elif ARGS.scenario == "lost_alt":
    check("안전(lost_alt): 영구 소실 뒤 HEARTBEAT 가 끊기면(30 s~) 모드를 모르므로 autonomous_land=1 이어도 FAILSAFE_LAND 에서 LAND 를 보내지 않는다",
          main.AUTONOMOUS_LAND and not [c for c in FC.mode_calls if c[2] == "LAND"] and state_at(36.0) in ("FAILSAFE_LAND", "WAIT_LEADER"),
          f"autonomous_land={main.AUTONOMOUS_LAND} modes={FC.mode_calls} state@36={state_at(36.0)}")
elif ARGS.scenario == "takeover":
    _land = [c for c in FC.mode_calls if c[2] == "LAND"]
    check("안전(takeover): HEARTBEAT 1 Hz 에서 조종사가 34.6 s 에 LOITER 로 탈환하면 35.0 s 의 FAILSAFE_LAND 결정은 직전 GUIDED heartbeat(34.03 s) 만으로 LAND 를 보내지 않는다 — 결정 뒤 heartbeat 가 LOITER 라 영영 안 보냄, 조종사 모드 유지",
          main.AUTONOMOUS_LAND and not _land and FC.mode == "LOITER" and first_time("FAILSAFE_LAND", 30.0) is not None,
          f"autonomous_land={main.AUTONOMOUS_LAND} modes={FC.mode_calls} fc_mode={FC.mode} t_fs={first_time('FAILSAFE_LAND', 30.0)}")

if ARGS.scenario != "default":
    print()
    print(f"{'FAILED: ' + ', '.join(failures) if failures else '폐루프 검사 전부 통과'} ({len(failures)} 실패)")
    sys.exit(1 if failures else 0)

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
_dmin = min(d for t, d, _ in World.dist if 11.0 <= t <= 16.0)
# 리더는 3.0s 에 출발, FOLLOW 확정은 ~5.1s 라 그 사이 0.6m 가 벌어진 채 시작한다. 같은 조건에서 P 만이면 +1.15m.
check("FOLLOW 중(리더 0.3m/s, t=10.5s) 거리 오차 < 0.7m — 피드포워드 (KFF=0 이면 +1.15m, 정상상태 v/Kp=1.36m. FF 저역통과 2s·데드존 0.05 뒤 정상상태 0.45m)",
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
t_fs = first_time("FAILSAFE_LAND", 30.0)
check("영구 소실 25s → 10초 뒤 FAILSAFE_LAND 상태", t_fs is not None and 34.0 <= t_fs <= 36.5, f"t={t_fs}")
if main.AUTONOMOUS_LAND:
    check("영구 소실 25s → 10초 뒤 LAND 1회", len(land) == 1 and 34.0 <= land[0][1] <= 36.5, f"{land}")
    check("LAND 이후 FC 가 하강 중 (가짜 FC 가 LAND 를 받아들임)", land and World.f_d > -15.0 + 1.0, f"d={World.f_d:.2f}")
else:
    _hold = _sp_between(35.5, 39.9)
    check("정책(autonomous_land=False, 기본): FAILSAFE_LAND 에서 LAND 를 보내지 않고 0 속도 setpoint 를 10 Hz 로 계속 보내며(35.5~40 s ≥ 40개, 전부 0) FC 는 고도를 유지한다",
          not land and len(_hold) >= 40 and all(abs(x) < 1e-12 for s in _hold for x in s[2:]) and abs(World.f_d + 15.0) < 0.05,
          f"land={land} hold_n={len(_hold)} d={World.f_d:.2f}")

stream = {
    "setpoints": [[f, t, round(vx, 9), round(vy, 9), round(vz, 9), round(yr, 9)] for f, t, vx, vy, vz, yr in FC.setpoints],
    "mode_calls": [list(c) for c in FC.mode_calls],
    "states": [list(s) for s in STATES],
    "final_pose": [round(World.f_n, 6), round(World.f_e, 6), round(World.f_d, 6), round(World.f_yaw, 6)],
    # 그림용 (analysis/readme_figures.py): 프레임별 (sim_t, 리더 거리, 리더 가시)
    "dist": [[round(t, 4), round(d, 4), int(v)] for t, d, v in World.dist],
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
