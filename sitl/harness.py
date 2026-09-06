#!/usr/bin/env python3
"""MARS-IMM SITL 회귀 하네스.

저장소의 **실제 `main.main()`을** ArduCopter SITL에 붙여 비행시킨다. 스텁은 인지 계층
(cv2 / RealSense / YOLO)뿐이고, 미션 상태머신 · 제어 · MAVLink 송신은 전부 실제 코드가 돈다.
`SEND_MAVLINK_COMMANDS=True`로 실행되므로 SITL 기체가 실제로 움직인다.

SITL 준비는 sitl/README.md 참조. 요약:

    arducopter -I0 --model + --speedup 1 --defaults <copter.parm> \\
        --home 35.83,128.75,50,0 \\
        --serial0 udpclient:127.0.0.1:14551 \\
        --serial1 udpclient:127.0.0.1:14552

사용:

    python3 sitl/harness.py --scenario boot_no_leader
    python3 sitl/harness.py --all

차등 검증 — 수정 전 코드에서 결함이 실제로 재현되는지 확인한다. 대조군이 통과하는
테스트는 아무것도 증명하지 못한다:

    git worktree add --detach /tmp/before HEAD~1
    python3 sitl/harness.py --all --repo /tmp/before
"""

import argparse
import os
import sys
import threading
import time
import types

# --repo를 main import보다 먼저 처리해야 하므로 argparse를 최상단에서 돌린다.
_P = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
_P.add_argument("--scenario", default="boot_no_leader",
                choices=["boot_no_leader", "pilot_takeover", "hold_heading",
                         "air_landing", "depth_range", "hover_hold", "px4_setmode"])
_P.add_argument("--all", action="store_true", help="모든 시나리오를 순서대로")
_P.add_argument("--repo", default=str(__import__("pathlib").Path(__file__).resolve().parent.parent),
                help="검사할 저장소 경로 (대조군은 수정 전 worktree를 지정)")
_P.add_argument("--fc-port", default="udpin:0.0.0.0:14551", help="main.py가 붙을 SITL 포트")
_P.add_argument("--pilot-port", default="udpin:0.0.0.0:14552", help="감시/조종사 링크 포트")
_P.add_argument("--alt", type=float, default=15.0, help="이륙 고도 [m]")
_P.add_argument("--duration", type=float, default=35.0, help="시나리오 길이 [s]")
_P.add_argument("--leader-front", type=float, default=4.5, help="리더 전방 거리 [m]")
_P.add_argument("--force-target", type=float, default=None,
                help="TARGET_DISTANCE_M 강제. 두 arm의 이동량을 통제할 때 사용 (C5 격리)")
_P.add_argument("--wp-yaw-behavior", type=int, default=0,
                help="기체 파라미터. C5 재현 시도는 2(공장 기본값)")
ARGS = _P.parse_args()

sys.path.insert(0, ARGS.repo)

import numpy as np  # noqa: E402

W, H = 640, 480
FX = FY = 384.0
CX, CY = W / 2.0, H / 2.0


# ----------------------------------------------------------------- 인지 스텁
class _Cv2(types.ModuleType):
    FONT_HERSHEY_SIMPLEX = 0

    def __getattr__(self, name):        # imshow / rectangle / putText / ... 전부 no-op
        return lambda *a, **k: 0


sys.modules["cv2"] = _Cv2("cv2")
sys.modules["pyrealsense2"] = types.ModuleType("pyrealsense2")
_u = types.ModuleType("ultralytics")
_u.YOLO = object
sys.modules["ultralytics"] = _u
sys.modules["serial"] = types.ModuleType("serial")

import camera        # noqa: E402
import detector      # noqa: E402


class World:
    """리더/팔로워의 월드 상태.

    폐루프다: 팔로워가 실제로 움직이면 리더까지의 거리가 변한다. 이게 있어야
    정위치 유지(H2)나 거리 추종(C4)이 관측 가능해진다. 팔로워 위치/기수는
    watcher가 SITL의 LOCAL_POSITION_NED / ATTITUDE로 갱신한다.
    """
    visible = True
    frames = 0
    stop = False

    # 팔로워 (local NED, m / rad)
    f_n = f_e = f_d = 0.0
    f_yaw = 0.0
    have_fix = False

    # 리더 오프셋 — 팔로워 초기 위치 기준 NED. 시나리오가 조작한다.
    l_n = l_e = l_d = 0.0

    @classmethod
    def reset(cls):
        cls.visible = True
        cls.frames = 0
        cls.stop = False
        cls.have_fix = False
        cls.f_n = cls.f_e = cls.f_d = 0.0
        cls.f_yaw = 0.0
        # 기본: 정북 방향 leader_front 앞, 같은 고도
        cls.l_n, cls.l_e, cls.l_d = ARGS.leader_front, 0.0, 0.0

    @classmethod
    def relative_fru(cls):
        """리더의 팔로워 기준 (front, right, up) [m]."""
        dn = cls.l_n - cls.f_n
        de = cls.l_e - cls.f_e
        dd = cls.l_d - cls.f_d
        c, s = np.cos(cls.f_yaw), np.sin(cls.f_yaw)
        front = dn * c + de * s
        right = -dn * s + de * c
        up = -dd
        return front, right, up


def in_fov():
    u, v, _, front = _bbox_px()
    return World.visible and front > 0.3 and 0 <= u < W and 0 <= v < H


def _bbox_px():
    front, right, up = World.relative_fru()
    front = max(front, 0.2)
    u = int(CX + FX * (right / front))
    v = int(CY - FY * (up / front))
    half = max(18, int(0.35 * FX / front))     # 폭 0.7m 물체
    return u, v, half, front


class FakeCam:
    def __init__(self, *a, **k):
        self.depth_scale = 0.001
        self.intrinsics = {"fx": FX, "fy": FY, "ppx": CX, "ppy": CY}

    def start(self):
        print("[CAM] fake D435i (SITL harness)")

    def stop(self):
        pass

    def get_frames(self):
        World.frames += 1
        if World.stop:
            raise KeyboardInterrupt("scenario finished")
        time.sleep(1.0 / 30.0)
        color = np.zeros((H, W, 3), dtype=np.uint8)
        # 배경은 15m — 어떤 depth_max보다도 멀어서 리더만 유효 픽셀이 된다.
        depth = np.full((H, W), int(15.0 / self.depth_scale), dtype=np.uint16)
        if World.visible:
            u, v, half, front = _bbox_px()
            if 0 <= u < W and 0 <= v < H:
                depth[max(0, v - half):v + half, max(0, u - half):u + half] = int(
                    front / self.depth_scale
                )
        return color, depth


class FakeDetector:
    def __init__(self, *a, **k):
        pass

    def detect(self, image, roi=None):
        if not World.visible:
            return []
        u, v, half, _ = _bbox_px()
        if not (0 <= u < W and 0 <= v < H):
            return []                       # FOV 밖이면 안 보인다
        return [{
            "bbox": [float(u - half), float(v - half), float(u + half), float(v + half)],
            "conf": 0.85,
            "cls_name": "person",
        }]


camera.D435i = FakeCam
detector.YoloDetector = FakeDetector
detector.load_model = lambda *a, **k: None

os.environ["MARS_FC_PORT"] = ARGS.fc_port
import main         # noqa: E402
import mavlink_io   # noqa: E402
from pymavlink import mavutil  # noqa: E402

main.D435i = FakeCam
main.YoloDetector = FakeDetector
main.SHOW_WINDOW = False
main.USE_LEADER_ESP32 = False        # ESP32 송신 펌웨어가 없으므로 비전 단독 경로
main.SEND_MAVLINK_COMMANDS = True    # 회귀의 핵심: 실제로 FC에 명령이 나간다
main.CONFIG["logger"]["enabled"] = False
mavlink_io.SERIAL_PORT = ARGS.fc_port

if ARGS.force_target is not None:
    main.TARGET_DISTANCE_M = ARGS.force_target


# ----------------------------------------------------------------- 유틸
T0 = time.time()


def log(msg):
    print(f"[{time.time() - T0:6.1f}s] {msg}", flush=True)


def request_streams(m):
    """mavlink_io.connect_fc()가 하는 것과 동일. 없으면 위치/자세가 안 온다."""
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_ALL,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
        m.mav.request_data_stream_send(m.target_system, m.target_component, sid, 10, 1)


def preflight(m):
    """GUIDED 진입 → ARM → 이륙. ArduCopter는 이 순서여야 한다."""
    m.mav.param_set_send(m.target_system, m.target_component, b"WP_YAW_BEHAVIOR",
                         ARGS.wp_yaw_behavior, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    time.sleep(0.5)

    m.set_mode("GUIDED")
    time.sleep(2)

    t0 = time.time()
    while time.time() - t0 < 180:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == 0:
            break
        time.sleep(3)
    else:
        raise RuntimeError("ARM 실패 (EKF/GPS 준비 안 됨?)")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, ARGS.alt)
    m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

    t0 = time.time()
    while time.time() - t0 < 90:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg and msg.relative_alt / 1000.0 >= ARGS.alt * 0.9:
            log(f"preflight 완료: {msg.relative_alt / 1000.0:.1f}m, GUIDED")
            return
    raise RuntimeError("이륙 실패 (고도 미도달)")


# ----------------------------------------------------------------- 시나리오
def run_px4_setmode():
    """C6 — PX4의 mode_mapping은 3-튜플이다. set_mode가 그걸 견디는가.

    이 결함은 전송 **전** 패킹에서 터진다(uint32 필드에 튜플). 따라서 비행도 EKF도
    필요 없고, PX4로 식별되는 MAVLink 엔드포인트만 있으면 재현된다.
    """
    global T0
    T0 = time.time()
    ok = True

    m = mavutil.mavlink_connection(ARGS.fc_port)
    hb = m.wait_heartbeat(timeout=60)
    ap = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
    log(f"연결: autopilot={ap}")
    if "PX4" not in ap:
        log(f"!! 이 엔드포인트는 PX4가 아니다 ({ap}). C6는 PX4에서만 발생한다.")
        return False

    mapping = m.mode_mapping() or {}
    land_key = "AUTO.LAND" if "AUTO.LAND" in mapping else "LAND"
    val = mapping.get(land_key)
    log(f"mode_mapping['{land_key}'] = {val!r} (type={type(val).__name__})")
    if not isinstance(val, tuple):
        log("!! 3-튜플이 아니다 — 이 pymavlink/기체 조합에서는 C6 전제가 성립하지 않는다")
        return False

    for key in (land_key, "LAND"):
        if key not in mapping:
            continue
        try:
            r = main.set_mode(m, key)
            log(f"set_mode({key}) -> {r} (예외 없음)")
        except Exception as e:
            log(f"!! FAIL C6: set_mode({key})가 {type(e).__name__}: {e}")
            ok = False

    try:
        main.send_land(m)
        log("send_land() 완료 (예외 없음)")
    except Exception as e:
        log(f"!! FAIL C6: send_land()가 {type(e).__name__}: {e}")
        ok = False

    m.close()
    print(f"{'PASS' if ok else 'FAIL'} (px4_setmode)")
    return ok


def run_scenario(name):
    if name == "px4_setmode":
        return run_px4_setmode()

    global T0
    T0 = time.time()
    World.reset()
    verdict = {"pass": True, "why": []}

    def fail(why):
        verdict["pass"] = False
        verdict["why"].append(why)
        log(f"!! FAIL: {why}")

    pilot = mavutil.mavlink_connection(ARGS.pilot_port)
    pilot.wait_heartbeat(timeout=60)
    request_streams(pilot)
    preflight(pilot)

    # 시계는 preflight가 끝난 뒤부터. 이륙에 걸린 시간이 시나리오 예산을 먹으면 안 된다.
    T0 = time.time()

    # C3: 절대(대지) 고도를 못 얻는 상황을 재현한다. LOCAL_POSITION_NED는 EKF origin
    # 설정 전에는 실제로 오지 않으며, 그때 leader_alt_est가 None이 된다.
    # 로직을 건드리는 게 아니라 "메시지가 없는 환경"을 만드는 것이다.
    if name == "air_landing":
        _real_state = main.get_vehicle_state

        def _no_local_position():
            st = dict(_real_state())
            st["local_position"] = {}
            return st

        main.get_vehicle_state = _no_local_position
        log("LOCAL_POSITION_NED 차단 → leader_alt_est=None 조건 재현")

    seen_modes, headings, ranges = [], [], []
    land_seen_at = [None]
    took_over_at = [None]

    def watcher():
        while not World.stop:
            msg = pilot.recv_match(blocking=True, timeout=1.0)
            now = time.time() - T0
            if now > ARGS.duration:
                World.stop = True
                break
            if msg is None:
                continue
            t = msg.get_type()

            # 폐루프 월드: 팔로워가 실제로 움직인 만큼 리더까지의 거리가 변한다
            if t == "LOCAL_POSITION_NED":
                World.f_n, World.f_e, World.f_d = float(msg.x), float(msg.y), float(msg.z)
                if not World.have_fix:
                    World.l_n = World.f_n + ARGS.leader_front
                    World.l_e = World.f_e
                    World.l_d = World.f_d
                    World.have_fix = True
                    log(f"월드 기준점: 팔로워 NED=({World.f_n:.1f},{World.f_e:.1f},"
                        f"{World.f_d:.1f}), 리더 전방 {ARGS.leader_front}m")
            elif t == "ATTITUDE":
                World.f_yaw = float(msg.yaw)

            if t == "VFR_HUD" and name == "hold_heading":
                headings.append((now, float(msg.heading)))

            if t == "LOCAL_POSITION_NED":
                front, _, _ = World.relative_fru()
                ranges.append((now, front, -World.f_d))

            if t != "HEARTBEAT":
                continue
            mode = mavutil.mode_string_v10(msg)
            if not seen_modes or seen_modes[-1][1] != mode:
                seen_modes.append((now, mode))
                log(f"FC mode -> {mode}")

            if name in ("boot_no_leader", "air_landing"):
                if mode == "LAND" and land_seen_at[0] is None:
                    land_seen_at[0] = now
                    agl = -World.f_d
                    if name == "boot_no_leader":
                        fail(f"C1: 리더를 한 번도 못 봤는데 LAND 전환 (t={now:.1f}s)")
                    elif in_fov():
                        fail(f"C3: 리더가 보이는데 고도 {agl:.1f}m 공중에서 "
                             f"착륙 판정 → LAND (t={now:.1f}s)")
                    else:
                        # 리더가 FOV를 벗어난 뒤의 LAND는 정당한 소실 failsafe다
                        log(f"(리더 FOV 이탈 후 LAND — 정당한 failsafe, C3 아님)")

            elif name == "pilot_takeover":
                if mode == "LAND" and land_seen_at[0] is None:
                    land_seen_at[0] = now
                    log("FAILSAFE_LAND 관측 → 조종사 LOITER 탈환")
                    pilot.set_mode("LOITER")
                    took_over_at[0] = now
                elif took_over_at[0] is not None and mode == "LAND" \
                        and now - took_over_at[0] > 0.5:
                    fail(f"C2: 조종사 탈환이 {now - took_over_at[0]:.1f}s 만에 LAND로 덮어써짐")
                    World.stop = True

    def driver():
        if name == "boot_no_leader":
            World.visible = False
            log("시나리오: 리더 미획득 상태로 대기 (C1)")
        elif name == "pilot_takeover":
            log("시나리오: 리더 소실 → FAILSAFE_LAND → 조종사 탈환 (C2)")
            time.sleep(8)
            World.visible = False
            log("리더 소실 (lost_hold 5s 후 FAILSAFE_LAND 예상)")
        elif name == "hold_heading":
            log("시나리오: 리더 고정, 기수 드리프트 감시 (C5)")

        elif name == "air_landing":
            # LOCAL_POSITION_NED가 없으면 leader_alt_est=None이 되고,
            # 수정 전 코드는 상대 z로 대체해 공중에서 착륙 판정을 통과시킨다.
            log("시나리오: 절대고도 없음 + 리더 하강 → 공중 착륙 판정 여부 (C3)")
            while not World.stop and not World.have_fix:
                time.sleep(0.2)
            # 6초만 하강한다. landing_confirm_sec(1.8s)를 넘기기엔 충분하고,
            # 계속 내리면 리더가 FOV를 벗어나 "정당한" 리더 소실 failsafe가 걸려
            # 착륙 판정과 구분할 수 없게 된다.
            t0 = time.time()
            while not World.stop and time.time() - t0 < 6.0:
                World.l_d += 0.3 * 0.1     # 0.3 m/s 하강 (팔로워 MAX_VZ 0.12보다 빠름)
                time.sleep(0.1)
            log("하강 종료, 리더 고도 유지")

        elif name == "depth_range":
            # 정지한 리더는 FOLLOW를 유발하지 않는다(hspeed > start_speed_thresh 필요).
            # 리더를 0.3 m/s로 계속 전진시켜 "따라붙을 수 있는가"를 본다.
            # P 제어라 평형 거리 = TARGET + v/KP_FORWARD 이므로, 그 값이 depth_max를
            # 넘으면 거리 관측을 잃고 무한히 뒤처진다 — 이것이 C4다.
            log("시나리오: 리더 0.3 m/s 전진 — 평형거리가 깊이창 안에 드는가 (C4)")
            while not World.stop and not World.have_fix:
                time.sleep(0.2)
            while not World.stop:
                World.l_n += 0.3 * 0.1
                time.sleep(0.1)

        elif name == "hover_hold":
            log("시나리오: 리더 전진 후 정지 → 팔로워가 정위치를 유지하는가 (H2)")
            while not World.stop and not World.have_fix:
                time.sleep(0.2)
            t0 = time.time()
            # 1단계: 0.3 m/s로 8초 전진 → FOLLOW 진입 + 팔로워가 뒤처진 상태를 만든다
            while not World.stop and time.time() - t0 < 8:
                World.l_n += 0.3 * 0.1
                time.sleep(0.1)
            log(f"리더 정지. 이후 팔로워가 목표거리로 수렴하는지 관측")

    def runner():
        try:
            main.main()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            fail(f"main.main() 예외: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
        World.stop = True

    threads = [threading.Thread(target=f, daemon=True) for f in (runner, watcher, driver)]
    for t in threads:
        t.start()

    deadline = time.time() + ARGS.duration + 10
    while time.time() < deadline and not World.stop:
        time.sleep(0.5)
    World.stop = True
    time.sleep(1.5)

    log(f"mode 이력: {[f'{t:.0f}s:{s}' for t, s in seen_modes]}")
    if os.environ.get("HARNESS_TRACE"):
        for t, r, agl in ranges[::10]:
            log(f"  t={t:5.1f}s front={r:5.2f}m agl={agl:5.1f}m fov={in_fov()}")

    if name in ("depth_range", "hover_hold") and len(ranges) > 10:
        target = float(main.TARGET_DISTANCE_M)
        settle = [r for t, r, _ in ranges if t > ARGS.duration * 0.6]
        final = sum(settle) / len(settle) if settle else ranges[-1][1]
        peak = max(r for _, r, _ in ranges)
        log(f"리더까지 거리: 최대 {peak:.1f}m → 후반 평균 {final:.1f}m "
            f"(목표 {target:.1f}m)")
        if name == "depth_range":
            # 평형 거리 = TARGET + v/KP_FORWARD. v=0.3, KP=0.22 → +1.4m.
            # 여유를 둬서 target+3.0을 넘으면 따라붙지 못한 것으로 본다.
            if final > target + 3.0:
                fail(f"C4: 리더 0.3 m/s를 따라붙지 못함 (후반 {final:.1f}m, "
                     f"목표 {target:.1f}m). 평형거리가 깊이창 밖이면 거리 관측을 잃는다")
        else:
            if abs(final - target) > 1.5:
                fail(f"H2: 리더 정지 후 목표거리 {target:.1f}m로 수렴하지 못함 "
                     f"(후반 {final:.1f}m). 정위치 유지가 동작하지 않는다")

    if name == "hold_heading":
        late = [h for t, h in headings if t > 10.0]
        if len(late) > 5:
            drift = max(abs((h - late[0] + 180) % 360 - 180) for h in late)
            log(f"기수 최대 편차 {drift:.1f}° ({len(late)}샘플)")
            if drift > 25.0:
                fail(f"C5: 명령 yaw_rate=0인데 기수가 {drift:.1f}° 자체 회전")
        else:
            log("기수 샘플 부족 — 판정 불가")

    pilot.close()
    print(f"프레임 {World.frames}개 · {'PASS' if verdict['pass'] else 'FAIL'} ({name})")
    if verdict["why"]:
        print("  " + " / ".join(verdict["why"]))
    return verdict["pass"]


if __name__ == "__main__":
    names = ["boot_no_leader", "pilot_takeover", "air_landing",
             "depth_range", "hover_hold", "hold_heading"] if ARGS.all \
        else [ARGS.scenario]
    print(f"저장소: {ARGS.repo}")
    results = {}
    for n in names:
        print(f"\n{'=' * 70}\n=== {n} ===")
        try:
            results[n] = run_scenario(n)
        except Exception as e:
            print(f"시나리오 준비 실패: {type(e).__name__}: {e}")
            results[n] = False

    print(f"\n{'=' * 70}")
    for n, ok in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {n}")
    sys.exit(0 if all(results.values()) else 1)
