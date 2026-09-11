"""
main.py — MARS-IMM Drone Follow Refactor 메인 루프

포함 기능:
- D435i RGB-D
- YOLO11n / TensorRT detector
- LeaderTracker
- MeasurementBuilder
- ReliabilityEstimator
- Risk-bound PerceptionScheduler
- IMM-EKF
- ESP32 leader telemetry 수신
- MissionManager
- Pixhawk MAVLink BODY_NED velocity setpoint

ESP32 선두 telemetry 가정:
- lat, lon, alt
- vx, vy, vz
- roll, pitch, yaw
- timestamp

중요:
- 기본값 SEND_MAVLINK_COMMANDS=False
- 처음에는 명령을 보내지 않고 로그/화면만 확인
- 실제 명령 송신 전 반드시 프로펠러 제거 상태에서 QGC/Mission Planner로 setpoint 확인
"""

import os

import cv2
import time
import math
import numpy as np

from pymavlink import mavutil

from config import CONFIG
from camera import D435i
from detector import YoloDetector
from tracker import LeaderTracker
from measurement import MeasurementBuilder
from reliability import ReliabilityEstimator
from scheduler import PerceptionScheduler
from imm_ekf import ImmEkf
from utils_geometry import bbox_center
from logger import ExperimentLogger

from mavlink_io import (
    connect_fc,
    drain_messages,
    get_vehicle_state,
    battery_text,
)

from mission_manager import MissionManager, S_WAIT_LEADER

from leader_telemetry import (
    LeaderTelemetryReceiver,
    build_leader_measurement_from_packet,
    apply_leader_velocity_hint_to_imm,
)


# ============================================================
# 실행 설정
# ============================================================

SHOW_WINDOW = os.environ.get("MARS_SHOW_WINDOW", "1") != "0"

# 처음에는 반드시 False.
# True로 바꾸기 전:
# 1. 프로펠러 제거
# 2. Pixhawk mode 확인
# 3. QGC/Mission Planner에서 velocity setpoint 확인
# 4. 저속 제한으로 테스트
SEND_MAVLINK_COMMANDS = False

USE_MARS_IMM_DEFAULT = True
USE_BEARING_FALLBACK = True

# ESP32 leader telemetry 사용.
# serial.Serial()이 SerialException을 던지고, 그 호출이 try 블록 밖이라
# 루프 진입 전에 프로그램이 죽는다.
USE_LEADER_ESP32 = True

# ESP32 수신 방식
# "serial" 또는 "udp"
LEADER_TELEMETRY_KIND = "serial"

# serial 사용 시
LEADER_SERIAL_PORT = "/dev/ttyUSB0"
LEADER_SERIAL_BAUD = 115200

# udp 사용 시
LEADER_UDP_IP = "0.0.0.0"
LEADER_UDP_PORT = 5005

# ESP32 packet freshness
LEADER_MAX_AGE_SEC = 0.70

# 선두 velocity 좌표계
# ESP32가 vx=east, vy=north, vz=up 으로 보내면 "ENU"
# ESP32가 vx=north, vy=east, vz=down 으로 보내면 "NED"
LEADER_VELOCITY_FRAME = "ENU"

# 목표 추종 거리
TARGET_DISTANCE_M = 3.0  # C4: depth_max 10m 대비 여유 7m.
# P제어 정상상태 평형거리 = TARGET + v_leader/KP_FORWARD 이므로
# 이 값과 config의 depth_max_m 간격이 곧 추종 가능한 리더 속도 상한이다.
# (5.0 + depth_max 6.0 조합에서는 리더 0.22 m/s에서 이미 깊이창 밖이었다)

# BODY_NED velocity limit
# BODY_NED:
#   x = forward +
#   y = right +
#   z = down +
MAX_VX = 0.35
MAX_VY = 0.22
MAX_VZ = 0.12

# 첫 실비행에서는 더 낮게 시작 권장
# MAX_VX = 0.15
# MAX_VY = 0.08
# MAX_VZ = 0.05

# controller gain
KP_FORWARD = 0.22
KD_FORWARD = 0.05

KP_RIGHT = 0.28
KD_RIGHT = 0.04

KP_UP = 0.18
KD_UP = 0.03

# yaw 제어: 선두를 카메라 시야(FOV ~69도) 중앙에 유지하기 위한 bearing 추종.
# bearing(rad) 오차 → yaw_rate(rad/s)
KP_YAW = 0.8
MAX_YAW_RATE = 0.35

UNCERTAINTY_SLOWDOWN_TRACE = CONFIG["controller"].get(
    "uncertainty_slowdown_trace",
    4.0,
)

SETPOINT_PERIOD_SEC = 0.10
# C2: LAND가 먹지 않았을 때만 이 간격으로 재시도. 100ms 연타는 조종사 탈환을 덮어쓴다.
LAND_RETRY_SEC = 2.0
# 카메라 프레임을 연속 이 횟수만큼 못 받으면 포기한다(약 CAM_FAIL_LIMIT/FPS 초).
CAM_FAIL_LIMIT = 30
# 이 고도(AGL) 아래에서는 하강 명령을 내지 않는다.
MIN_AGL_M = 1.5

GPS_MAX_AGE_SEC = 0.70
LOCAL_POS_MAX_AGE_SEC = 0.40
ATTITUDE_MAX_AGE_SEC = 0.30


# ============================================================
# 화면 유틸
# ============================================================

def put_text(frame, text, pos, color=(255, 255, 0), scale=0.60, thickness=2):
    cv2.putText(
        frame,
        text,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
    )


def depth_color(depth_m):
    if depth_m is None:
        return (150, 150, 150)
    if depth_m < 2.5:
        return (0, 0, 255)
    if depth_m <= 4.5:
        return (0, 255, 0)
    return (255, 165, 0)


def draw_model_bar(frame, mu, x=20, y=262, w=180):
    labels = ["CV", "CT"]
    colors = [(255, 200, 50), (50, 200, 255)]
    bar_h = 12

    for i, (label, color, prob) in enumerate(zip(labels, colors, mu)):
        by = y + i * (bar_h + 5)
        cv2.rectangle(frame, (x, by), (x + w, by + bar_h), (50, 50, 50), -1)
        cv2.rectangle(frame, (x, by), (x + int(w * prob), by + bar_h), color, -1)
        put_text(
            frame,
            f"{label} {prob:.2f}",
            (x + w + 6, by + bar_h - 1),
            color,
            scale=0.48,
            thickness=1,
        )


# ============================================================
# 좌표 변환
# ============================================================

def camera_xyz_to_fru(x_cam):
    """
    IMM-EKF 상태:
        camera XYZ = [right, down, forward]

    MissionManager / controller:
        FRU = [front, right, up]
    """
    x_cam = np.asarray(x_cam, dtype=float)
    if x_cam.size < 3:
        return np.zeros(3, dtype=float)

    right = float(x_cam[0])
    down = float(x_cam[1])
    front = float(x_cam[2])
    up = -down

    return np.array([front, right, up], dtype=float)


def fru_to_body_ned_velocity(v_fru):
    """
    FRU velocity:
        [forward, right, up]

    BODY_NED velocity:
        [forward, right, down]
    """
    v_fru = np.asarray(v_fru, dtype=float)
    return np.array(
        [
            float(v_fru[0]),
            float(v_fru[1]),
            -float(v_fru[2]),
        ],
        dtype=float,
    )


def get_follower_altitude_m(vehicle_state):
    """
    LOCAL_POSITION_NED:
        z는 down 방향.
        고도는 -z.
    """
    lp = vehicle_state.get("local_position", {})
    z = lp.get("z", None)
    if z is None:
        return None
    try:
        return -float(z)
    except Exception:
        return None


def is_fresh(msg, now, max_age):
    if not msg:
        return False
    ts = msg.get("timestamp", 0.0)
    try:
        ts = float(ts)
    except Exception:
        return False
    if ts <= 0.0:
        return False
    return (now - ts) <= max_age


# ============================================================
# MAVLink velocity / mode 명령
# ============================================================

def send_body_velocity(master, vx, vy, vz, yaw_rate=0.0):
    """
    BODY_NED velocity setpoint 송신.

    vx: forward + [m/s]
    vy: right + [m/s]
    vz: down + [m/s]
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    # C5: YAW_RATE_IGNORE를 세우면 기수 유지를 FC의 WP_YAW_BEHAVIOR에 위임하게 된다.
    # yaw_rate=0.0 + 비트 clear = "현재 기수 유지"를 명시하는 쪽이 옳고 비용이 0이라
    # 항상 mask 1479를 쓴다.
    # 단, 이전 감사가 보고한 "기수 자동회전으로 164.6도 이탈"은 SITL에서 재현되지
    # 않았고 발생할 수 없음이 확인됐다 — LOOK_AHEAD는 진입 시 현재 기수로 초기화되고
    # 지면속도 1 m/s 초과에서만 갱신되는데, 이 포크는 버그 마스크를 명령 속도가 0일
    # 때만 내보낸다. 자세한 근거는 sitl/README.md 참조.

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0.0, 0.0, 0.0,
        float(vx), float(vy), float(vz),
        0.0, 0.0, 0.0,
        0.0,
        float(yaw_rate),
    )


def send_hold(master):
    send_body_velocity(master, 0.0, 0.0, 0.0, 0.0)


def set_mode(master, mode_name):
    mode_mapping = master.mode_mapping()
    if mode_mapping is None or mode_name not in mode_mapping:
        print(f"[WARN] mode {mode_name} not available in mode_mapping")
        return False

    # C6: PX4의 mode_mapping 값은 3-튜플(예: LAND -> (29, 4, 6))이라
    # set_mode_send의 uint32 필드에 넣으면 struct.error가 나고, 예외 처리가
    # 없어 비행 중 제어 루프가 죽는다. master.set_mode()가 apm/px4를 자동 분기한다.
    try:
        master.set_mode(mode_name)
    except Exception as e:
        print(f"[WARN] set_mode({mode_name}) 실패: {type(e).__name__}: {e}")
        return False

    print(f"[FC] set mode: {mode_name}")
    return True


def send_land(master):
    # ArduPilot: "LAND" / PX4: "AUTO.LAND"
    for mode_name in ("LAND", "AUTO.LAND"):
        if set_mode(master, mode_name):
            return

    print("[FC] send MAV_CMD_NAV_LAND")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    )


# ============================================================
# 제어기
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, pos_cov_trace):
    """
    MARS-IMM 추정값을 BODY_NED velocity command로 변환.

    rel_fru:
        [front, right, up]

    rel_vel_fru:
        [front_vel, right_vel, up_vel]

    목표:
        front = TARGET_DISTANCE_M
        right = 0
        up = 0
    """
    rel_fru = np.asarray(rel_fru, dtype=float)
    rel_vel_fru = np.asarray(rel_vel_fru, dtype=float)

    front = float(rel_fru[0])
    right = float(rel_fru[1])
    up = float(rel_fru[2])

    v_front = float(rel_vel_fru[0])
    v_right = float(rel_vel_fru[1])
    v_up = float(rel_vel_fru[2])

    front_err = front - TARGET_DISTANCE_M
    right_err = right
    up_err = up

    cmd_forward = KP_FORWARD * front_err + KD_FORWARD * v_front
    cmd_right = KP_RIGHT * right_err + KD_RIGHT * v_right
    cmd_up = KP_UP * up_err + KD_UP * v_up

    if pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE:
        scale = 0.55
    elif pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE * 0.5:
        scale = 0.75
    else:
        scale = 1.0

    cmd_forward *= scale
    cmd_right *= scale
    cmd_up *= scale

    cmd_forward = clamp(cmd_forward, -MAX_VX, MAX_VX)
    cmd_right = clamp(cmd_right, -MAX_VY, MAX_VY)
    cmd_up = clamp(cmd_up, -MAX_VZ, MAX_VZ)

    # yaw: 선두 방위각(bearing)을 0으로 유지 → 카메라 시야 이탈 방지.
    # BODY_NED yaw_rate는 우회전 +, bearing도 타겟이 오른쪽이면 + → 부호 일치.
    bearing_err = math.atan2(right, max(front, 0.5))
    cmd_yaw_rate = clamp(KP_YAW * bearing_err * scale, -MAX_YAW_RATE, MAX_YAW_RATE)

    body_vel = fru_to_body_ned_velocity([cmd_forward, cmd_right, cmd_up])
    return np.append(body_vel, cmd_yaw_rate)


SMOOTH_REF_DT = 1.0 / 30.0   # alpha가 튜닝된 기준 프레임 간격


def smooth_velocity_cmd(prev_cmd, new_cmd, alpha=0.28, dt=None):
    """cmd = [body_vx, body_vy, body_vz, yaw_rate]

    alpha는 프레임당 값이라 그대로 쓰면 평활 시정수가 FPS에 딸려간다
    (24fps와 30fps에서 응답이 25% 다르다). dt를 주면 기준 30fps 기준으로
    보정해 실제 FPS와 무관하게 같은 시정수를 갖는다.
    """
    prev_cmd = np.asarray(prev_cmd, dtype=float)
    new_cmd = np.asarray(new_cmd, dtype=float)

    a = float(alpha)
    if dt is not None and dt > 0:
        a = 1.0 - (1.0 - a) ** (float(dt) / SMOOTH_REF_DT)
        a = clamp(a, 0.0, 1.0)

    out = (1.0 - a) * prev_cmd + a * new_cmd

    out[0] = clamp(out[0], -MAX_VX, MAX_VX)
    out[1] = clamp(out[1], -MAX_VY, MAX_VY)
    out[2] = clamp(out[2], -MAX_VZ, MAX_VZ)
    out[3] = clamp(out[3], -MAX_YAW_RATE, MAX_YAW_RATE)

    return out


# ============================================================
# 메인
# ============================================================

def main():
    global SEND_MAVLINK_COMMANDS

    print("[SYS] start MARS-IMM Drone Follow")

    use_mars_imm = USE_MARS_IMM_DEFAULT

    detector = YoloDetector()

    cam = D435i(
        width=CONFIG["camera"]["width"],
        height=CONFIG["camera"]["height"],
        fps=CONFIG["camera"]["fps"],
    )
    cam.start()

    intrinsics = cam.intrinsics
    print(
        f"[CAM] fx={intrinsics['fx']:.1f} "
        f"fy={intrinsics['fy']:.1f} "
        f"ppx={intrinsics['ppx']:.1f} "
        f"ppy={intrinsics['ppy']:.1f}"
    )

    meas_builder = MeasurementBuilder(intrinsics, depth_scale=cam.depth_scale)
    reliability = ReliabilityEstimator()
    scheduler = PerceptionScheduler()
    tracker = LeaderTracker()
    ekf = ImmEkf()
    mission = MissionManager()

    master = connect_fc()

    leader_rx = None
    if USE_LEADER_ESP32:
        leader_rx = LeaderTelemetryReceiver(
            kind=LEADER_TELEMETRY_KIND,
            port=LEADER_SERIAL_PORT,
            baud=LEADER_SERIAL_BAUD,
            udp_ip=LEADER_UDP_IP,
            udp_port=LEADER_UDP_PORT,
        )
        try:
            leader_rx.start()
        except Exception as exc:
            # 장치가 없거나 pyserial이 없어도 비전 단독 경로로 계속 간다.
            print(f"[WARN] leader telemetry 비활성: {type(exc).__name__}: {exc}")
            leader_rx = None

    log = ExperimentLogger(CONFIG["logger"]["log_dir"]) if CONFIG["logger"]["enabled"] else None

    last_track = None
    last_depth_m = None
    last_h_ratio = None
    last_ex = 0.0

    prev_time = time.time()
    last_bat_print = 0.0
    last_setpoint_time = 0.0
    last_land_send = 0.0  # C2: LAND 재시도 타이머
    prev_fc_accepts = False  # GUIDED 진입 에지 검출용
    cam_fail_streak = 0      # 연속 카메라 실패 횟수
    show_window = SHOW_WINDOW

    fps_counter = 0
    fps_t0 = time.time()
    fps_display = 0.0

    prev_body_cmd = np.zeros(4, dtype=float)
    current_body_cmd = np.zeros(4, dtype=float)
    prev_yaw_for_comp = None

    print("=" * 90)
    print("[INFO] q/ESC 종료 | m: MARS-IMM on/off | v: MAVLink velocity 송신 on/off")
    print("[INFO] l: LAND 명령 | h: HOLD 명령")
    print(f"[INFO] SEND_MAVLINK_COMMANDS={SEND_MAVLINK_COMMANDS}")
    print(f"[INFO] USE_LEADER_ESP32={USE_LEADER_ESP32}, kind={LEADER_TELEMETRY_KIND}")
    print("[INFO] 실제 비행 전 반드시 프로펠러 제거 상태에서 확인")
    print("=" * 90)

    try:
        while True:
            now = time.time()
            dt = max(now - prev_time, 1e-4)
            prev_time = now

            # ------------------------------------------------------------
            # Pixhawk / ESP32 수신
            # ------------------------------------------------------------
            drain_messages(master)
            vehicle_state = get_vehicle_state()

            # C2: FC 모드를 매 루프 읽는다. 이전에는 1Hz 출력 블록 안에서만 읽어
            # 송신 지점에서 참조할 수 없었다. GUIDED/OFFBOARD를 벗어났다는 것은
            # 조종사가 수동 탈환했다는 뜻이므로 모든 송신을 하드 스톱한다.
            fc_mode = vehicle_state.get("mode", {}).get("name", "?")
            fc_armed = vehicle_state.get("mode", {}).get("armed", False)
            fc_accepts_setpoints = fc_mode in ("GUIDED", "OFFBOARD")

            # GUIDED 진입 = 조종사가 방금 자동에게 넘긴 순간. 그 전까지 FC는 우리
            # 명령을 전부 버렸으므로, 그동안 쌓인 상태를 그대로 들고 들어가면 안 된다.
            #
            #  - 미션: 수동 상승 중에는 리더가 화면 밖이라 10초 뒤 FAILSAFE_LAND로
            #    래치된다. 리셋하지 않으면 스위치를 넘기는 첫 프레임에 LAND가 나간다.
            #    (C1 가드는 "한 번도 못 봄"만 막고 "상승 중 놓침"은 막지 못한다)
            #  - 평활 버퍼: FC가 무시하는 동안 목표값까지 수렴해 있으므로, 리셋하지
            #    않으면 전환 첫 setpoint가 램프 없이 포화 상태로 나간다.
            if fc_accepts_setpoints and not prev_fc_accepts:
                mission.state = S_WAIT_LEADER
                mission.last_seen_t = None
                mission.first_seen_t = None
                mission.start_candidate_t = None
                mission.landing_candidate_t = None
                prev_body_cmd = np.zeros(4, dtype=float)
                last_land_send = 0.0
                print(f"[SYS] {fc_mode} 진입 — 미션/명령 리셋")
            prev_fc_accepts = fc_accepts_setpoints

            gps_fresh = is_fresh(vehicle_state.get("gps", {}), now, GPS_MAX_AGE_SEC)
            local_pos_fresh = is_fresh(vehicle_state.get("local_position", {}), now, LOCAL_POS_MAX_AGE_SEC)
            attitude_fresh = is_fresh(vehicle_state.get("attitude", {}), now, ATTITUDE_MAX_AGE_SEC)

            leader_packet = leader_rx.read_latest() if leader_rx is not None else None

            leader_meas = build_leader_measurement_from_packet(
                packet=leader_packet,
                follower_vehicle_state=vehicle_state,
                now=now,
                max_age_sec=LEADER_MAX_AGE_SEC,
                leader_velocity_frame=LEADER_VELOCITY_FRAME,
            )

            if now - last_bat_print >= 1.0:
                p_cv, p_ct = ekf.get_model_probs() if ekf.initialized else (0.0, 0.0)
                print(
                    f"[STAT] {battery_text()} "
                    f"FPS={fps_display:.1f} "
                    f"MARS={'ON' if use_mars_imm else 'OFF'} "
                    f"CMD={'ON' if SEND_MAVLINK_COMMANDS else 'DRY'} "
                    f"FC={fc_mode}{'*' if fc_armed else ''} "
                    f"ESP={int(leader_meas.get('available', False))}:{leader_meas.get('reason', 'none')} "
                    f"CV={p_cv:.2f} CT={p_ct:.2f} "
                    f"coast={ekf.coast_time:.1f}s rcoast={ekf.range_coast_time:.1f}s "
                    f"mission={mission.state}"
                )

                # velocity setpoint는 ArduPilot GUIDED / PX4 OFFBOARD에서만 동작
                if SEND_MAVLINK_COMMANDS and not fc_accepts_setpoints:
                    print(
                        f"[WARN] FC mode={fc_mode}: 송신 중단됨 "
                        f"(ArduPilot: GUIDED / PX4: OFFBOARD 필요)"
                    )
                last_bat_print = now

            # ------------------------------------------------------------
            # Camera
            # ------------------------------------------------------------
            # 카메라 예외(USB hiccup, 'Frame didn't arrive within 5000')는 흔하고,
            # 잡지 않으면 비행 중에 제어 루프가 통째로 죽는다. 드롭 프레임으로
            # 처리하고, 연속으로 실패하면 그때 포기한다.
            try:
                color_image, depth_image = cam.get_frames()
                cam_fail_streak = 0
            except Exception as exc:
                cam_fail_streak += 1
                print(f"[WARN] 카메라 프레임 실패 {cam_fail_streak}회: "
                      f"{type(exc).__name__}: {exc}")
                if cam_fail_streak >= CAM_FAIL_LIMIT:
                    print(f"[ERR] 카메라 연속 실패 {CAM_FAIL_LIMIT}회 — 종료")
                    break
                continue

            if color_image is None:
                print("[WARN] frame dropped")
                continue

            H, W = color_image.shape[:2]
            cx_img = W // 2

            fps_counter += 1
            if now - fps_t0 >= 1.0:
                fps_display = fps_counter / max(now - fps_t0, 1e-6)
                fps_counter = 0
                fps_t0 = now

            # ------------------------------------------------------------
            # IMM predict
            # ------------------------------------------------------------
            # 후미 기체가 yaw하면 카메라 프레임이 회전하므로,
            # 실제 yaw 변화량만큼 상대상태를 역회전시켜 보정한다.
            # (yaw 제어를 켠 상태에서 이 보정이 없으면 predict가 계속 틀어짐)
            cur_yaw = vehicle_state.get("attitude", {}).get("yaw", None)
            if attitude_fresh and cur_yaw is not None:
                cur_yaw = float(cur_yaw)
                if prev_yaw_for_comp is not None and ekf.initialized:
                    dpsi = math.atan2(
                        math.sin(cur_yaw - prev_yaw_for_comp),
                        math.cos(cur_yaw - prev_yaw_for_comp),
                    )
                    ekf.compensate_ego_yaw(dpsi)
                prev_yaw_for_comp = cur_yaw
            else:
                prev_yaw_for_comp = None

            if ekf.initialized:
                ekf.predict(dt)

            # ------------------------------------------------------------
            # Perception scheduler
            # ------------------------------------------------------------
            if use_mars_imm:
                policy = scheduler.decide(
                    imm_state=ekf.get_state_dict(),
                    image_shape=color_image.shape,
                    intrinsics=intrinsics,
                    last_track=last_track,
                )
            else:
                policy = {
                    "run_detector": True,
                    "use_full_frame": True,
                    "roi": None,
                    "detect_every": 1,
                    "reason": "baseline_full_frame",
                }

            # ------------------------------------------------------------
            # Detection / Tracking
            # ------------------------------------------------------------
            if policy["run_detector"]:
                detections = detector.detect(
                    color_image,
                    roi=None if policy["use_full_frame"] else policy["roi"],
                )
                track = tracker.update(detections)
            else:
                detections = []
                track = tracker.predict_only()

            last_track = track

            # ------------------------------------------------------------
            # Measurement build
            # ------------------------------------------------------------
            rgbd_meas = meas_builder.build_rgbd(track, depth_image)
            bearing_meas = meas_builder.build_bearing(track) if track is not None else None

            r_vis = reliability.vision_reliability(rgbd_meas or bearing_meas or track)
            r_depth = reliability.depth_reliability(rgbd_meas)

            update_used = "none"
            gate_d2 = None

            # ------------------------------------------------------------
            # Vision / RGB-D IMM update
            # ------------------------------------------------------------
            if rgbd_meas is not None and r_vis > 0.0 and r_depth > 0.0:
                R = reliability.make_R_rgbd(r_vis, r_depth) if use_mars_imm else None

                if not ekf.initialized:
                    ekf.init(rgbd_meas["z"])
                    update_used = "init_rgbd"
                else:
                    gate_ok, gate_d2 = reliability.gate_position3d(
                        ekf,
                        rgbd_meas["z"],
                        R if R is not None else None,
                    )

                    if gate_ok or not use_mars_imm:
                        ekf.update_position3d(rgbd_meas["z"], R)
                        update_used = "rgbd"
                    elif USE_BEARING_FALLBACK and bearing_meas is not None:
                        Rb = reliability.make_R_bearing(r_vis)
                        gate_ok_b, gate_d2 = reliability.gate_bearing2d(
                            ekf,
                            bearing_meas["z"],
                            Rb,
                        )
                        if gate_ok_b:
                            ekf.update_bearing2d(bearing_meas["z"], Rb)
                            update_used = "bearing_after_rgbd_gate"
                        else:
                            update_used = "gate_reject_all"
                    else:
                        update_used = "gate_reject_rgbd"

            elif USE_BEARING_FALLBACK and bearing_meas is not None and ekf.initialized:
                Rb = reliability.make_R_bearing(r_vis)
                gate_ok_b, gate_d2 = reliability.gate_bearing2d(
                    ekf,
                    bearing_meas["z"],
                    Rb,
                )
                if gate_ok_b:
                    ekf.update_bearing2d(bearing_meas["z"], Rb)
                    update_used = "bearing"
                else:
                    update_used = "gate_reject_bearing"

            # ------------------------------------------------------------
            # ESP32 leader GPS / velocity fusion
            # ------------------------------------------------------------
            esp_update_used = "none"
            esp_vel_hint_used = False
            esp_gate_d2 = None
            r_esp_gps = 0.0
            r_esp_time = 0.0

            if USE_LEADER_ESP32 and leader_meas.get("available", False):
                z_esp = np.asarray(leader_meas["rel_cam"], dtype=float)

                age = float(leader_meas.get("age", 999.0))
                r_esp_time = float(np.exp(-age / max(LEADER_MAX_AGE_SEC, 1e-6)))

                # 현재 packet에 GPS quality 정보가 없다고 가정.
                # 나중에 fix_type, satellites, h_acc를 넣으면 reliability.gps_reliability로 교체.
                r_esp_gps = max(0.05, 0.8 * r_esp_time)
                R_esp = reliability.make_R_gps(r_esp_gps)

                if not ekf.initialized:
                    ekf.init(z_esp)
                    esp_update_used = "init_esp_gps"
                else:
                    gate_ok_esp, esp_gate_d2 = reliability.gate_position3d(
                        ekf,
                        z_esp,
                        R_esp,
                    )

                    if gate_ok_esp:
                        ekf.update_position3d(z_esp, R_esp)
                        esp_update_used = "esp_gps"
                    else:
                        esp_update_used = "gate_reject_esp_gps"

                esp_vel_hint_used = apply_leader_velocity_hint_to_imm(
                    ekf,
                    leader_meas["rel_vel_cam"],
                    alpha=0.10,
                    shrink_vel_cov=0.98,
                )

            # ------------------------------------------------------------
            # lost 처리
            # ------------------------------------------------------------
            if track is None or track.get("is_lost", False):
                if ekf.initialized:
                    ekf.on_lost(dt)

            # ------------------------------------------------------------
            # EKF state
            # ------------------------------------------------------------
            x_est, P_est = ekf.get_state()
            pos_cov_trace = float(np.trace(P_est[:3, :3])) if ekf.initialized else 999.0
            mu = ekf.get_model_probs() if ekf.initialized else np.array([0.0, 0.0])

            rel_fru = camera_xyz_to_fru(x_est[:3])
            rel_vel_fru = camera_xyz_to_fru(x_est[3:6])

            follower_alt = get_follower_altitude_m(vehicle_state) if local_pos_fresh else None

            # 착륙 판정용 선두 고도는 AGL 근사값이어야 한다.
            # ESP32 packet.alt는 WGS84 절대고도(수십~수백 m)라
            # landing_z_thresh(~0.65m)와 직접 비교하면 착륙 감지가 절대 발동하지 않음.
            # → follower AGL(LOCAL_POSITION_NED 기준) + 상대고도로 계산.
            if follower_alt is not None and ekf.initialized and ekf.is_reliable():
                leader_alt_est = float(follower_alt + rel_fru[2])
            else:
                leader_alt_est = None

            if leader_meas.get("available", False):
                # 미션(출발/호버/착륙) 판단은 선두의 "절대" 속도 기준
                leader_vel_world = np.asarray(leader_meas["leader_vel_enu"], dtype=float)
                leader_hspeed = float(leader_meas.get("leader_hspeed", 0.0))
                leader_vz_up = float(leader_meas.get("leader_vz_up", 0.0))
            else:
                leader_vel_world = None
                leader_hspeed = None
                leader_vz_up = None

            # mission에서 선두가 보인다고 볼 조건:
            # - tracker가 직접 보임
            # - EKF가 신뢰 가능
            # - ESP32 leader telemetry가 정상
            track_visible = track is not None and not track.get("is_lost", False)
            ekf_reliable = ekf.initialized and ekf.is_reliable()
            esp_visible = bool(leader_meas.get("available", False))

            # 미션의 "리더가 보인다"는 판정은 bbox가 아니라 **거리를 아는가**여야 한다.
            # track_visible은 YOLO가 상자만 그려도 참이 되고, ekf.is_reliable()은
            # bearing-only 업데이트로도 참이 된다. 둘 다 거리 관측을 보장하지 않으므로
            # 깊이가 죽어도 소실 판정이 나지 않아 실패 착륙이 발동하지 않는다.
            # RGB-D와 ESP32 위치만 거리를 담으며, 그 둘만 range_coast_time을 되돌린다.
            leader_visible_for_mission = bool(ekf.has_range_fix())

            # ------------------------------------------------------------
            # Mission state manager
            # ------------------------------------------------------------
            mission_state, mission_policy = mission.update(
                now=now,
                leader_visible=leader_visible_for_mission,
                rel_est=rel_fru if ekf.initialized else None,
                rel_vel_est=rel_vel_fru if ekf.initialized else None,
                leader_alt=leader_alt_est,
                leader_vel_world=leader_vel_world,
                pos_cov_trace=pos_cov_trace,
                manual_override=False,
            )

            # ------------------------------------------------------------
            # Command decision
            # ------------------------------------------------------------
            desired_body_cmd = np.zeros(4, dtype=float)

            if mission_policy["land"]:
                desired_body_cmd = np.zeros(4, dtype=float)

                # C2: 이전에는 100ms마다 set_mode("LAND")를 재송신해서
                # 조종사가 LOITER로 탈환해도 100~135ms 만에 다시 끌려갔다.
                #
                # 모드 게이트가 곧 latch다: LAND가 실제로 먹으면 FC 모드가
                # GUIDED/OFFBOARD를 벗어나므로 이 분기가 더 이상 실행되지 않는다.
                # 여전히 여기 있다는 건 명령이 먹지 않았다는 뜻이라 그때만
                # LAND_RETRY_SEC 간격으로 재시도한다 (조종사 탈환 시엔
                # fc_accepts_setpoints=False라 아예 보내지 않는다).
                if SEND_MAVLINK_COMMANDS and fc_accepts_setpoints:
                    if now - last_land_send >= LAND_RETRY_SEC:
                        send_land(master)
                        last_land_send = now
                        last_setpoint_time = now

            elif mission_policy["allow_follow"] and ekf.initialized:
                desired_body_cmd = compute_velocity_cmd_from_estimate(
                    rel_fru=rel_fru,
                    rel_vel_fru=rel_vel_fru,
                    pos_cov_trace=pos_cov_trace,
                )

            else:
                desired_body_cmd = np.zeros(4, dtype=float)

            # 수직 축은 리더의 상대 고도를 따라간다. 리더가 아래에 있으면(혹은
            # 트래커가 지면의 무언가를 물면) 계속 하강 명령이 나가고, 멈추는 조건은
            # "리더 고도에 도달"뿐이다. AGL 바닥을 둔다.
            if follower_alt is not None and follower_alt < MIN_AGL_M:
                if desired_body_cmd[2] > 0.0:          # BODY_NED z는 down +
                    desired_body_cmd[2] = 0.0

            current_body_cmd = smooth_velocity_cmd(
                prev_cmd=prev_body_cmd,
                new_cmd=desired_body_cmd,
                alpha=0.28,
                dt=dt,
            )
            prev_body_cmd = current_body_cmd.copy()

            # setpoint 송신
            #
            # 속도 setpoint에는 모드 게이트를 걸지 않는다. GUIDED/OFFBOARD가 아니면
            # FC가 조용히 버리므로(조종사를 방해하지 않음), 반대로 PX4는 OFFBOARD에
            # 진입하기 전에 이 스트림이 먼저 흐르고 있어야 한다. 게이트를 걸면
            # PX4에서는 영원히 OFFBOARD에 못 들어가는 데드락이 된다.
            # 조종사를 뺏는 것은 setpoint가 아니라 모드 변경(LAND)이며, 그쪽만 막는다.
            if not mission_policy["land"]:
                last_land_send = 0.0
                if now - last_setpoint_time >= SETPOINT_PERIOD_SEC:
                    if SEND_MAVLINK_COMMANDS:
                        send_body_velocity(
                            master,
                            current_body_cmd[0],
                            current_body_cmd[1],
                            current_body_cmd[2],
                            yaw_rate=current_body_cmd[3],
                        )
                    last_setpoint_time = now

            # ------------------------------------------------------------
            # Display target
            # ------------------------------------------------------------
            raw_depth_m = rgbd_meas.get("depth_m") if rgbd_meas else None
            ekf_depth_m = float(x_est[2]) if ekf.initialized and ekf.is_reliable() else None

            if track is not None and not track.get("is_lost", False):
                x1, y1, x2, y2 = track["bbox"]
                cx_tgt, cy_tgt = bbox_center(track["bbox"])
                ex = (cx_tgt - cx_img) / max(cx_img, 1)

                bh = y2 - y1
                h_ratio = bh / max(H, 1)

                ctrl_depth = ekf_depth_m if ekf_depth_m is not None else raw_depth_m
                last_depth_m = ctrl_depth
                last_h_ratio = h_ratio
                last_ex = ex

                if show_window:
                    col = depth_color(ctrl_depth)
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), col, 2)
                    cv2.circle(color_image, (int(cx_tgt), int(cy_tgt)), 5, (0, 0, 255), -1)

                    raw_s = f"{raw_depth_m:.2f}m" if raw_depth_m is not None else "N/A"
                    ekf_s = f"{ekf_depth_m:.2f}m" if ekf_depth_m is not None else "N/A"
                    put_text(
                        color_image,
                        f"raw={raw_s} est={ekf_s} {update_used}",
                        (x1, max(22, y1 - 8)),
                        col,
                        scale=0.48,
                    )

            # ------------------------------------------------------------
            # 화면 출력
            # ------------------------------------------------------------
            if show_window:
                if not policy.get("use_full_frame", True) and policy.get("roi") is not None:
                    rx1, ry1, rx2, ry2 = policy["roi"]
                    cv2.rectangle(color_image, (rx1, ry1), (rx2, ry2), (255, 0, 255), 1)

                cv2.line(color_image, (cx_img, 0), (cx_img, H), (0, 255, 255), 1)

                put_text(
                    color_image,
                    f"MARS:{'ON' if use_mars_imm else 'OFF'} CMD:{'ON' if SEND_MAVLINK_COMMANDS else 'DRY'}",
                    (20, 26),
                )
                put_text(color_image, battery_text(), (20, 54), (0, 255, 255))
                put_text(
                    color_image,
                    f"FPS={fps_display:.1f} policy={policy.get('reason')}",
                    (20, 82),
                    (200, 200, 200),
                    scale=0.50,
                )
                put_text(
                    color_image,
                    f"mission={mission_state} mode={mission_policy['mode']}",
                    (20, 110),
                    (100, 255, 255),
                    scale=0.50,
                )
                put_text(
                    color_image,
                    f"rV={r_vis:.2f} rD={r_depth:.2f} gate={gate_d2 if gate_d2 is not None else -1:.1f}",
                    (20, 138),
                    (180, 180, 255),
                    scale=0.50,
                )
                put_text(
                    color_image,
                    f"ESP={int(esp_visible)} {leader_meas.get('reason', 'none')} upd={esp_update_used}",
                    (20, 166),
                    (180, 220, 255),
                    scale=0.48,
                )
                put_text(
                    color_image,
                    f"rel F/R/U=({rel_fru[0]:+.2f},{rel_fru[1]:+.2f},{rel_fru[2]:+.2f}) cov={pos_cov_trace:.2f}",
                    (20, 194),
                    (220, 220, 220),
                    scale=0.48,
                )
                put_text(
                    color_image,
                    f"cmd BODY_NED vx={current_body_cmd[0]:+.2f} vy={current_body_cmd[1]:+.2f} vz={current_body_cmd[2]:+.2f} yr={current_body_cmd[3]:+.2f}",
                    (20, 222),
                    (100, 255, 100),
                    scale=0.48,
                )
                put_text(
                    color_image,
                    f"fresh GPS/LP/ATT={int(gps_fresh)}/{int(local_pos_fresh)}/{int(attitude_fresh)}",
                    (20, 250),
                    (180, 180, 255),
                    scale=0.48,
                )

                if ekf.initialized:
                    draw_model_bar(color_image, ekf.get_model_probs())

                try:
                    cv2.imshow("MARS-IMM Drone Follow", color_image)
                    key = cv2.waitKey(1) & 0xFF
                except Exception as exc:
                    # DISPLAY가 없거나 ssh -X 링크가 끊기면 imshow가 예외를 던진다.
                    # 창을 포기하고 헤드리스로 계속 난다 — 여기서 죽으면 안 된다.
                    print(f"[WARN] 화면 비활성화 ({type(exc).__name__}) — 헤드리스로 계속")
                    show_window = False
                    key = 255

                if key in (ord("q"), 27):
                    break

                elif key == ord("m"):
                    use_mars_imm = not use_mars_imm
                    print(f"[SYS] MARS-IMM -> {'ON' if use_mars_imm else 'OFF'}")

                elif key == ord("v"):
                    SEND_MAVLINK_COMMANDS = not SEND_MAVLINK_COMMANDS
                    print(f"[SYS] SEND_MAVLINK_COMMANDS -> {SEND_MAVLINK_COMMANDS}")

                elif key == ord("h"):
                    if SEND_MAVLINK_COMMANDS:
                        send_hold(master)
                        print("[SYS] manual HOLD 송신 (다음 setpoint에 덮임)")
                    else:
                        print("[SYS] manual HOLD 생략 — CMD=DRY")

                elif key == ord("l"):
                    if SEND_MAVLINK_COMMANDS:
                        send_land(master)
                        print("[SYS] manual LAND 송신")
                    else:
                        print("[SYS] manual LAND 생략 — CMD=DRY ('v'로 켜야 나감)")

            # ------------------------------------------------------------
            # Logging
            # ------------------------------------------------------------
            if log is not None:
                log.log(
                    {
                        "time": now,
                        "dt": dt,
                        "fps": fps_display,

                        "mars": {
                            "enabled": use_mars_imm,
                            "policy": policy,
                        },

                        "mission": {
                            "state": mission_state,
                            "mode": mission_policy["mode"],
                            "allow_follow": mission_policy["allow_follow"],
                            "land": mission_policy["land"],
                        },

                        "track": track or {},

                        "measurement": rgbd_meas or {},

                        "reliability": {
                            "vision": r_vis,
                            "depth": r_depth,
                            "gate_d2": gate_d2,
                            "update_used": update_used,
                        },

                        "leader_esp32": {
                            "available": leader_meas.get("available", False),
                            "reason": leader_meas.get("reason", "none"),
                            "age": leader_meas.get("age", 999.0),
                            "seq": leader_meas.get("seq", -1),

                            "leader_alt": leader_meas.get("leader_alt", None),
                            "leader_hspeed": leader_hspeed,
                            "leader_vz_up": leader_vz_up,

                            "roll": leader_meas.get("roll", None),
                            "pitch": leader_meas.get("pitch", None),
                            "yaw": leader_meas.get("yaw", None),

                            "rel_fru": leader_meas.get("rel_fru", np.zeros(3)),
                            "rel_cam": leader_meas.get("rel_cam", np.zeros(3)),
                            "rel_vel_fru": leader_meas.get("rel_vel_fru", np.zeros(3)),
                            "rel_vel_cam": leader_meas.get("rel_vel_cam", np.zeros(3)),

                            "r_esp_time": r_esp_time,
                            "r_esp_gps": r_esp_gps,
                            "esp_gate_d2": esp_gate_d2,
                            "esp_update_used": esp_update_used,
                            "esp_vel_hint_used": esp_vel_hint_used,
                        },

                        "ekf": {
                            "x": x_est,
                            "P_trace_pos": pos_cov_trace,
                            "mu": mu,
                            "coast_time": ekf.coast_time,
                            "initialized": ekf.initialized,
                            "reliable": ekf.is_reliable() if ekf.initialized else False,
                        },

                        "relative_fru": {
                            "front": rel_fru[0],
                            "right": rel_fru[1],
                            "up": rel_fru[2],
                            "v_front": rel_vel_fru[0],
                            "v_right": rel_vel_fru[1],
                            "v_up": rel_vel_fru[2],
                        },

                        "vehicle_state": {
                            "gps": vehicle_state.get("gps", {}),
                            "global_position": vehicle_state.get("global_position", {}),
                            "local_position": vehicle_state.get("local_position", {}),
                            "attitude": vehicle_state.get("attitude", {}),
                            "gps_fresh": gps_fresh,
                            "local_position_fresh": local_pos_fresh,
                            "attitude_fresh": attitude_fresh,
                        },

                        "control": {
                            "send_enabled": SEND_MAVLINK_COMMANDS,
                            "body_vx": current_body_cmd[0],
                            "body_vy": current_body_cmd[1],
                            "body_vz": current_body_cmd[2],
                            "yaw_rate": current_body_cmd[3],
                            "target_distance_m": TARGET_DISTANCE_M,
                        },
                    }
                )

    except KeyboardInterrupt:
        print("\n[SYS] KeyboardInterrupt")

    finally:
        print("[SYS] shutdown")

        try:
            if SEND_MAVLINK_COMMANDS:
                send_hold(master)
                time.sleep(0.1)
        except Exception as exc:
            print(f"[WARN] hold send failed: {exc}")

        try:
            if leader_rx is not None:
                leader_rx.close()
        except Exception:
            pass

        try:
            cam.stop()
        except Exception:
            pass

        if log is not None:
            log.close()

        cv2.destroyAllWindows()
        print("[SYS] done")


if __name__ == "__main__":
    main()
