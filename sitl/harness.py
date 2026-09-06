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
                choices=["boot_no_leader", "pilot_takeover", "hold_heading"])
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
    """시나리오가 조작하는 리더 상태 (후미 카메라 기준)."""
    visible = True
    front_m = 4.5
    right_m = 0.0
    up_m = 0.0
    frames = 0
    stop = False

    @classmethod
    def reset(cls):
        cls.visible = True
        cls.front_m = ARGS.leader_front
        cls.right_m = 0.0
        cls.up_m = 0.0
        cls.frames = 0
        cls.stop = False


def _bbox_px():
    u = int(CX + FX * (World.right_m / max(World.front_m, 0.1)))
    v = int(CY - FY * (World.up_m / max(World.front_m, 0.1)))
    half = max(18, int(0.35 * FX / max(World.front_m, 0.1)))   # 폭 0.7m 물체
    return u, v, half


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
        depth = np.full((H, W), int(9.5 / self.depth_scale), dtype=np.uint16)
        if World.visible:
            u, v, half = _bbox_px()
            depth[max(0, v - half):v + half, max(0, u - half):u + half] = int(
                World.front_m / self.depth_scale
            )
        return color, depth


class FakeDetector:
    def __init__(self, *a, **k):
        pass

    def detect(self, image, roi=None):
        if not World.visible:
            return []
        u, v, half = _bbox_px()
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
def run_scenario(name):
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

    seen_modes, headings = [], []
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

            if t == "VFR_HUD" and name == "hold_heading":
                headings.append((now, float(msg.heading)))

            if t != "HEARTBEAT":
                continue
            mode = mavutil.mode_string_v10(msg)
            if not seen_modes or seen_modes[-1][1] != mode:
                seen_modes.append((now, mode))
                log(f"FC mode -> {mode}")

            if name == "boot_no_leader":
                if mode == "LAND" and land_seen_at[0] is None:
                    land_seen_at[0] = now
                    fail(f"C1: 리더를 한 번도 못 봤는데 LAND 전환 (t={now:.1f}s)")

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
            log(f"시나리오: 리더 {World.front_m}m 고정, 기수 드리프트 감시 (C5)")

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
    names = ["boot_no_leader", "pilot_takeover", "hold_heading"] if ARGS.all \
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
