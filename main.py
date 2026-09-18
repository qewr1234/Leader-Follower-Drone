"""
main.py — MARS-IMM Drone Follow 메인 루프

D435i RGB-D → YOLO11n(TensorRT) → IoU 트래커 → IMM-EKF → 미션 상태머신 → P·D 제어 →
Pixhawk BODY_NED 속도 setpoint (10Hz). 단일 스레드 while 루프 하나.

- 기본값 SEND_MAVLINK_COMMANDS=False (dry-run). 실제 송신 전 프로펠러 제거 상태에서
  화면의 명령값을 확인하고, 조종기 스위치로 GUIDED 에 넣는 순간부터 추종이 시작된다.
- 조종사가 GUIDED/OFFBOARD 를 벗어나면 FC 가 setpoint 를 무시하고, 이 코드는 모드 변경
  (LAND)을 보내지 않는다 — 조종사가 항상 이긴다.
"""

import math
import os
import time

import cv2
import numpy as np
from pymavlink import mavutil

from camera import D435i
from config import CONFIG
from detector import YoloDetector
from imm_ekf import ImmEkf
from leader_telemetry import (LeaderTelemetryReceiver, apply_leader_velocity_hint_to_imm,
                              build_leader_measurement_from_packet)
from logger import ExperimentLogger
from mavlink_io import battery_text, connect_fc, drain_messages, get_vehicle_state, stream_rates_text
from measurement import MeasurementBuilder
from mission_manager import MissionManager
from reliability import ReliabilityEstimator
from scheduler import PerceptionScheduler
from tracker import LeaderTracker
from utils_geometry import bbox_center, clamp

# ============================================================
# 실행 설정
# ============================================================

SHOW_WINDOW = os.environ.get("MARS_SHOW_WINDOW", "1") != "0"
# 화면 출력(그리기+imshow+waitKey)은 비-YOLO 비용 중 가장 크다(Jetson 4~8ms). 2프레임에 한 번(15Hz)만
# 그린다. 키 입력(q/m/v/h/l)도 그 프레임에서만 읽히므로 최대 1프레임 늦게 반응한다.
DISPLAY_EVERY = 2

# 처음에는 반드시 False. True 로 바꾸기 전: 프로펠러 제거 → Pixhawk 모드 확인 → 화면 명령값 확인 → 저속.
SEND_MAVLINK_COMMANDS = False

USE_MARS_IMM_DEFAULT = True
USE_BEARING_FALLBACK = True

# ESP32 선두 텔레메트리. 장치가 없거나 pyserial 이 없으면 open_leader_receiver() 가 경고만 내고 None 을
# 돌려주므로 켜 둬도 비전 단독으로 간다. LEADER_SERIAL_PORT 에 다른 장치가 물려 있으면 그 바이트를 같이
# 읽게 되니(pyserial 은 배타 잠금이 없다) 포트 배정을 확인할 것.
USE_LEADER_ESP32 = True
LEADER_TELEMETRY_KIND = "serial"        # "serial" | "udp"
LEADER_SERIAL_PORT = "/dev/ttyUSB0"
LEADER_SERIAL_BAUD = 115200
LEADER_UDP_IP = "0.0.0.0"
LEADER_UDP_PORT = 5005
LEADER_MAX_AGE_SEC = 0.70
LEADER_VELOCITY_FRAME = "ENU"           # ESP32 가 vx=east,vy=north,vz=up 이면 ENU / north,east,down 이면 NED
# 패킷 "alt" 의 기준계. 필드명이 alt_msl / alt_ellipsoid 면 그쪽이 우선. 국내 지오이드 차이 ~25m 라 틀리면
# 그대로 상대 고도 오차가 된다.
LEADER_ALT_FRAME = "AMSL"

# 목표 추종 거리. P 제어 정상상태 평형거리 = TARGET + v_leader/KP_FORWARD 이므로 이 값과 config 의
# depth_max_m 간격이 곧 추종 가능한 리더 속도 상한이다 (C4).
TARGET_DISTANCE_M = 3.0
# 카메라 깊이 없이 ESP32 GPS 상대위치만으로 거리를 알 때의 이격. GPS 상대오차는 m 단위라 3m 는 오차보다
# 작다. 8m 는 depth_max(10m) 안이라 리더가 다시 깊이창에 들어오면 비전이 이어받는다.
TARGET_DISTANCE_GPS_ONLY_M = 8.0

# BODY_NED 속도 제한 (x=forward, y=right, z=down). 첫 실비행은 더 낮게(0.15/0.08/0.05) 권장.
MAX_VX = 0.35
MAX_VY = 0.22
MAX_VZ = 0.12

KP_FORWARD, KD_FORWARD = 0.22, 0.05
KP_RIGHT, KD_RIGHT = 0.28, 0.04
KP_UP, KD_UP = 0.18, 0.03
# yaw: 선두를 카메라 시야(FOV ~69도) 중앙에 유지. bearing(rad) 오차 → yaw_rate(rad/s)
KP_YAW = 0.8
MAX_YAW_RATE = 0.35

UNCERTAINTY_SLOWDOWN_TRACE = CONFIG["controller"].get("uncertainty_slowdown_trace", 4.0)
# 리더 속도 피드포워드 (config controller.leader_vel_ff_*)
KFF_LEADER_VEL = float(CONFIG["controller"].get("leader_vel_ff_gain", 0.8))
FF_TAU_SEC = float(CONFIG["controller"].get("leader_vel_ff_tau_sec", 0.7))
FF_DEADBAND_MPS = float(CONFIG["controller"].get("leader_vel_ff_deadband_mps", 0.10))

SETPOINT_PERIOD_SEC = 0.10
# C2: LAND 가 먹지 않았을 때만 이 간격으로 재시도. 100ms 연타는 조종사 탈환을 덮어쓴다.
LAND_RETRY_SEC = 2.0
CAM_FAIL_LIMIT = 30      # 카메라 연속 실패 한계. 스톨 1회 = camera 타임아웃 0.5s 라 약 15초 뒤 포기 (그동안 FC 의 GUID_TIMEOUT 이 먼저 든다)
FC_FAIL_LIMIT = 30       # FC 링크(drain) 연속 예외 한계
MIN_AGL_M = 1.5          # 이 고도(AGL) 아래에서는 하강 명령을 내지 않는다

GPS_MAX_AGE_SEC = 0.70
LOCAL_POS_MAX_AGE_SEC = 0.40
ATTITUDE_MAX_AGE_SEC = 0.30

SMOOTH_REF_DT = 1.0 / 30.0   # 평활 alpha 가 튜닝된 기준 프레임 간격

# ============================================================
# 좌표 / 상태 유틸
# ============================================================

def camera_xyz_to_fru(x_cam):
    """카메라 [right, down, forward] → FRU [front, right, up]."""
    x = np.asarray(x_cam, dtype=float)
    return np.array([x[2], x[0], -x[1]], dtype=float)


# 카메라 프레임(x=우, y=하, z=전) ← 기체 FRD(x=전, y=우, z=하)
_CAM_FROM_BODY = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])


def rot_body_to_ned(roll, pitch, yaw):
    """MAVLink ATTITUDE(ZYX 오일러) → 기체→NED 회전행렬 Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def ego_rotation_cam(prev_rpy, cur_rpy):
    """팔로워 자세가 prev→cur 로 바뀌었을 때, 이전 카메라 프레임의 벡터를 현재 카메라 프레임으로 옮기는 3x3.

    리더가 월드에 고정돼 있어도 기체가 돌면 카메라 안에서 움직여 보인다 — yaw 만 아니라 돌풍에 의한
    roll/pitch 도 마찬가지다(10° pitch ≈ 640px 화면에서 68px). p_b2 = R2ᵀ·R1·p_b1 을 카메라 프레임으로 옮긴 것.
    """
    d_rb = rot_body_to_ned(*cur_rpy).T @ rot_body_to_ned(*prev_rpy)
    return _CAM_FROM_BODY @ d_rb @ _CAM_FROM_BODY.T


def fru_to_body_ned_velocity(v_fru):
    """FRU [forward, right, up] → BODY_NED [forward, right, down]."""
    v = np.asarray(v_fru, dtype=float)
    return np.array([v[0], v[1], -v[2]], dtype=float)


def get_follower_altitude_m(vehicle_state):
    """LOCAL_POSITION_NED z 는 down 이므로 고도는 -z."""
    z = vehicle_state.get("local_position", {}).get("z")
    return None if z is None else -float(z)


def is_fresh(msg, now, max_age):
    ts = float(msg.get("timestamp", 0.0) or 0.0) if msg else 0.0
    return ts > 0.0 and (now - ts) <= max_age


# ============================================================
# MAVLink 송신
# ============================================================

# C5: YAW_RATE_IGNORE 를 세우면 기수 유지를 FC 의 WP_YAW_BEHAVIOR 에 위임하게 된다. yaw_rate=0.0 + 비트
# clear = "현재 기수 유지"를 명시하는 쪽이 옳고 비용이 0 이라 항상 마스크 1479 를 쓴다 (sitl/README.md).
_VEL_YAWRATE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


def send_body_velocity(master, vx, vy, vz, yaw_rate=0.0):
    """BODY_NED 속도 setpoint. vx forward+, vy right+, vz down+ [m/s], yaw_rate 우회전+ [rad/s]."""
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, _VEL_YAWRATE_MASK,
        0.0, 0.0, 0.0,
        float(vx), float(vy), float(vz),
        0.0, 0.0, 0.0,
        0.0, float(yaw_rate),
    )


def send_hold(master):
    send_body_velocity(master, 0.0, 0.0, 0.0, 0.0)


def set_mode(master, mode_name):
    mapping = master.mode_mapping()
    if mapping is None or mode_name not in mapping:
        print(f"[WARN] mode {mode_name} not available in mode_mapping")
        return False
    # C6: PX4 의 mode_mapping 값은 3-튜플이라 set_mode_send 의 uint32 필드에 넣으면 struct.error 가 난다.
    # master.set_mode() 가 apm/px4 를 자동 분기한다. 실패해도 루프는 살아야 한다.
    try:
        master.set_mode(mode_name)
    except Exception as e:
        print(f"[WARN] set_mode({mode_name}) 실패: {type(e).__name__}: {e}")
        return False
    print(f"[FC] set mode: {mode_name}")
    return True


def send_land(master):
    for mode_name in ("LAND", "AUTO.LAND"):        # ArduPilot / PX4
        if set_mode(master, mode_name):
            return
    print("[FC] send MAV_CMD_NAV_LAND")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# ============================================================
# 제어기
# ============================================================

def follower_velocity_fru(vehicle_state):
    """FC 의 LOCAL_POSITION_NED 속도(NED) 를 기체 yaw 로 돌린 (front, right, up). 값이 없으면 None."""
    lp = vehicle_state.get("local_position", {})
    yaw = vehicle_state.get("attitude", {}).get("yaw")
    vx, vy, vz = lp.get("vx"), lp.get("vy"), lp.get("vz")
    if None in (vx, vy, vz, yaw):
        return None
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    vx, vy, vz = float(vx), float(vy), float(vz)
    return np.array([vx * c + vy * s, -vx * s + vy * c, -vz])


def leader_velocity_ff(prev_ff, leader_vel_fru, dt):
    """피드포워드 항 갱신: 데드밴드(호버 잡음 억제) → 1차 저역통과(FF_TAU_SEC). leader_vel_fru 가 None 이면 0 으로 감쇠.

    리더 속도 = 자기 속도(FC 측정) + 상대 속도(EKF 추정). EKF 속도는 지연 δ 가 있어 자기 속도의 고주파 성분이
    s·δ/(1+s·δ) 이득으로 명령에 양성 되먹임된다. FC 속도루프 τ≈0.3s, δ≈1s 면 이 루프 이득 최대치가 약 0.77 이라
    KFF=1 이면 여유가 얇다. KFF 0.8 과 0.7s 저역통과를 곱하면 0.4 아래 — 대신 정상상태 오차 (1-KFF)·v/Kp 가 남는다.
    """
    target = np.zeros(3)
    if leader_vel_fru is not None:
        v = np.asarray(leader_vel_fru, dtype=float)
        speed = float(np.linalg.norm(v))
        if speed > FF_DEADBAND_MPS:
            target = v * clamp((speed - FF_DEADBAND_MPS) / FF_DEADBAND_MPS, 0.0, 1.0)
    prev = np.asarray(prev_ff, dtype=float)
    a = 1.0 - math.exp(-max(float(dt), 0.0) / max(FF_TAU_SEC, 1e-3))
    return prev + a * (target - prev)


def compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, pos_cov_trace, target_distance=None, leader_vel_ff=None):
    """IMM 추정(FRU 상대 위치·속도) → BODY_NED [vx, vy, vz, yaw_rate]. 네 축 모두 P+D + 리더 속도 피드포워드, 불확실하면 감속."""
    if target_distance is None:
        target_distance = TARGET_DISTANCE_M
    front, right, up = (float(v) for v in np.asarray(rel_fru, dtype=float)[:3])
    v_front, v_right, v_up = (float(v) for v in np.asarray(rel_vel_fru, dtype=float)[:3])

    if pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE:
        scale = 0.55
    elif pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE * 0.5:
        scale = 0.75
    else:
        scale = 1.0

    ff_f, ff_r, ff_u = (0.0, 0.0, 0.0) if leader_vel_ff is None else (float(v) for v in np.asarray(leader_vel_ff, dtype=float)[:3])
    cmd_forward = clamp((KFF_LEADER_VEL * ff_f + KP_FORWARD * (front - float(target_distance)) + KD_FORWARD * v_front) * scale, -MAX_VX, MAX_VX)
    cmd_right = clamp((KFF_LEADER_VEL * ff_r + KP_RIGHT * right + KD_RIGHT * v_right) * scale, -MAX_VY, MAX_VY)
    cmd_up = clamp((KFF_LEADER_VEL * ff_u + KP_UP * up + KD_UP * v_up) * scale, -MAX_VZ, MAX_VZ)
    # yaw: 선두 방위각을 0 으로 (시야 이탈 방지). BODY_NED yaw_rate 우회전 +, 타겟이 오른쪽이면 bearing + → 부호 일치.
    cmd_yaw_rate = clamp(KP_YAW * math.atan2(right, max(front, 0.5)) * scale, -MAX_YAW_RATE, MAX_YAW_RATE)

    return np.append(fru_to_body_ned_velocity([cmd_forward, cmd_right, cmd_up]), cmd_yaw_rate)


def smooth_velocity_cmd(prev_cmd, new_cmd, alpha=0.28, dt=None):
    """1차 평활. alpha 는 30fps 기준 프레임당 값이라 dt 를 주면 FPS 와 무관하게 같은 시정수를 갖는다."""
    a = float(alpha)
    if dt is not None and dt > 0:
        a = clamp(1.0 - (1.0 - a) ** (float(dt) / SMOOTH_REF_DT), 0.0, 1.0)
    out = (1.0 - a) * np.asarray(prev_cmd, dtype=float) + a * np.asarray(new_cmd, dtype=float)
    lim = (MAX_VX, MAX_VY, MAX_VZ, MAX_YAW_RATE)
    for i in range(4):
        out[i] = clamp(out[i], -lim[i], lim[i])
    return out


# ============================================================
# 센서 융합 (루프 본문에서 분리 — 전역은 호출 시점에 읽는다: 하네스가 import 후 패치)
# ============================================================

def _bearing_update(ekf, rel, bearing_meas, r_vis, ok_label, reject_label):
    Rb = rel.make_R_bearing(r_vis)
    gate_ok, d2 = rel.gate_bearing2d(ekf, bearing_meas["z"], Rb)
    if gate_ok:
        ekf.update_bearing2d(bearing_meas["z"], Rb)
        return ok_label, d2
    return reject_label, d2


def fuse_vision(ekf, rel, rgbd_meas, bearing_meas, r_vis, r_depth, use_mars_imm):
    """RGB-D 3D 측정 → 게이트 통과면 위치 업데이트, 아니면 bearing 으로 강등. 반환 (update_used, gate_d2)."""
    if rgbd_meas is not None and r_vis > 0.0 and r_depth > 0.0:
        R = rel.make_R_rgbd(r_vis, r_depth) if use_mars_imm else None
        if not ekf.initialized:
            ekf.init(rgbd_meas["z"])
            return "init_rgbd", None
        gate_ok, d2 = rel.gate_position3d(ekf, rgbd_meas["z"], R)
        if gate_ok or not use_mars_imm:
            ekf.update_position3d(rgbd_meas["z"], R)
            return "rgbd", d2
        if USE_BEARING_FALLBACK and bearing_meas is not None:
            return _bearing_update(ekf, rel, bearing_meas, r_vis, "bearing_after_rgbd_gate", "gate_reject_all")
        return "gate_reject_rgbd", d2
    if USE_BEARING_FALLBACK and bearing_meas is not None and ekf.initialized:
        return _bearing_update(ekf, rel, bearing_meas, r_vis, "bearing", "gate_reject_bearing")
    return "none", None


ESP_IDLE = {"esp_update_used": "none", "esp_vel_hint_used": False, "esp_gate_d2": None,
            "r_esp_gps": 0.0, "r_esp_time": 0.0}


def fuse_esp32(ekf, rel, leader_meas):
    """ESP32 GPS 상대위치를 게이트 후 위치 업데이트(source='gps'), 상대속도는 약한 힌트로.
    새 패킷일 때만 불린다 — 위치뿐 아니라 속도 힌트(alpha 0.10, P 수축 0.98)도 패킷 주기(5~10Hz)로 적용되므로
    실효 시정수는 프레임당 적용이던 때보다 길다. 같은 관측을 매 프레임 되풀이하는 것보다 이쪽이 맞다."""
    z_esp = np.asarray(leader_meas["rel_cam"], dtype=float)
    age = float(leader_meas.get("age", 999.0))
    r_esp_time = float(np.exp(-age / max(LEADER_MAX_AGE_SEC, 1e-6)))
    r_esp_gps = max(0.05, 0.8 * r_esp_time)     # 패킷에 GPS 품질 필드가 없어 신선도만으로
    R_esp = rel.make_R_gps(r_esp_gps)

    d2 = None
    if not ekf.initialized:
        ekf.init(z_esp, source="gps")
        used = "init_esp_gps"
    else:
        gate_ok, d2 = rel.gate_position3d(ekf, z_esp, R_esp)
        if gate_ok:
            ekf.update_position3d(z_esp, R_esp, source="gps")
        used = "esp_gps" if gate_ok else "gate_reject_esp_gps"

    hinted = apply_leader_velocity_hint_to_imm(ekf, leader_meas["rel_vel_cam"], alpha=0.10, shrink_vel_cov=0.98)
    return {"esp_update_used": used, "esp_vel_hint_used": hinted, "esp_gate_d2": d2,
            "r_esp_gps": r_esp_gps, "r_esp_time": r_esp_time}


def open_leader_receiver():
    """ESP32 수신기. 장치가 없거나 pyserial 이 없어도 예외를 내지 않고 None — 비전 단독으로 날아야 한다."""
    if not USE_LEADER_ESP32:
        return None
    rx = LeaderTelemetryReceiver(kind=LEADER_TELEMETRY_KIND, port=LEADER_SERIAL_PORT, baud=LEADER_SERIAL_BAUD,
                                 udp_ip=LEADER_UDP_IP, udp_port=LEADER_UDP_PORT, default_alt_frame=LEADER_ALT_FRAME)
    try:
        rx.start()
    except Exception as exc:
        print(f"[WARN] leader telemetry 비활성: {type(exc).__name__}: {exc}")
        return None
    return rx


# ============================================================
# 화면 / 로그
# ============================================================

def put_text(frame, text, pos, color=(255, 255, 0), scale=0.60, thickness=2):
    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)


def depth_color(depth_m):
    if depth_m is None:
        return (150, 150, 150)
    if depth_m < 2.5:
        return (0, 0, 255)
    if depth_m <= 4.5:
        return (0, 255, 0)
    return (255, 165, 0)


def draw_model_bar(frame, mu, x=20, y=262, w=180):
    for i, (label, color, prob) in enumerate(zip(("CV", "CT"), ((255, 200, 50), (50, 200, 255)), mu)):
        by = y + i * 17
        cv2.rectangle(frame, (x, by), (x + w, by + 12), (50, 50, 50), -1)
        cv2.rectangle(frame, (x, by), (x + int(w * prob), by + 12), color, -1)
        put_text(frame, f"{label} {prob:.2f}", (x + w + 6, by + 11), color, scale=0.48, thickness=1)


def draw_hud(frame, lines):
    """lines: (text, color, scale) 목록. 28px 간격으로 위에서부터."""
    for i, (text, color, scale) in enumerate(lines):
        put_text(frame, text, (20, 26 + 28 * i), color, scale)


def build_log_row(s):
    """s: 루프의 프레임 상태 dict. 키 이름은 로그 분석 스크립트가 의존하므로 바꾸지 않는다."""
    lm, esp, cmd = s["leader_meas"], s["esp"], s["current_body_cmd"]
    avail = lm.get("available", False)
    ekf = s["ekf"]
    return {
        "time": s["now"], "dt": s["dt"], "fps": s["fps_display"],
        "mars": {"enabled": s["use_mars_imm"], "policy": s["policy"]},
        "mission": {"state": s["mission_state"], "mode": s["mission_policy"]["mode"],
                    "allow_follow": s["mission_policy"]["allow_follow"], "land": s["mission_policy"]["land"]},
        "track": s["track"] or {},
        "measurement": s["rgbd_meas"] or {},
        "reliability": {"vision": s["r_vis"], "depth": s["r_depth"], "gate_d2": s["gate_d2"], "update_used": s["update_used"]},
        "leader_esp32": {
            "available": avail, "reason": lm.get("reason", "none"), "age": lm.get("age", 999.0), "seq": lm.get("seq", -1),
            "leader_alt": lm.get("leader_alt", None),
            "leader_hspeed": float(lm.get("leader_hspeed", 0.0)) if avail else None,
            "leader_vz_up": float(lm.get("leader_vz_up", 0.0)) if avail else None,
            "roll": lm.get("roll", None), "pitch": lm.get("pitch", None), "yaw": lm.get("yaw", None),
            "rel_fru": lm.get("rel_fru", np.zeros(3)), "rel_cam": lm.get("rel_cam", np.zeros(3)),
            "rel_vel_fru": lm.get("rel_vel_fru", np.zeros(3)), "rel_vel_cam": lm.get("rel_vel_cam", np.zeros(3)),
            **esp,
        },
        "ekf": {"x": s["x_est"], "P_trace_pos": s["pos_cov_trace"], "mu": s["mu"], "coast_time": ekf.coast_time,
                "range_coast_time": ekf.range_coast_time, "vision_range_coast_time": ekf.vision_range_coast_time,
                "initialized": ekf.initialized, "reliable": ekf.is_reliable() if ekf.initialized else False},
        "relative_fru": dict(zip(("front", "right", "up"), s["rel_fru"]), **dict(zip(("v_front", "v_right", "v_up"), s["rel_vel_fru"]))),
        "vehicle_state": {**{k: s["vehicle_state"].get(k, {}) for k in ("gps", "global_position", "local_position", "attitude")},
                          "gps_fresh": s["gps_fresh"], "local_position_fresh": s["local_pos_fresh"], "attitude_fresh": s["attitude_fresh"]},
        "control": {"send_enabled": SEND_MAVLINK_COMMANDS, "body_vx": cmd[0], "body_vy": cmd[1], "body_vz": cmd[2],
                    "yaw_rate": cmd[3], "target_distance_m": s["target_distance_m"], "vision_range_ok": s["vision_range_ok"],
                    "ff_front": s["ff_fru"][0], "ff_right": s["ff_fru"][1], "ff_up": s["ff_fru"][2]},
    }


# ============================================================
# 메인
# ============================================================

def main():
    global SEND_MAVLINK_COMMANDS

    print("[SYS] start MARS-IMM Drone Follow")
    use_mars_imm = USE_MARS_IMM_DEFAULT

    master = connect_fc()          # FC 먼저 — 없으면 YOLO 를 올리고 카메라를 켠 채 기다리지 않게.
    detector = YoloDetector()
    cam_cfg = CONFIG["camera"]
    warmup = getattr(detector, "warmup", None)     # 하네스의 FakeDetector 에는 없다
    if warmup:
        warmup(cam_cfg["width"], cam_cfg["height"])   # 엔진 역직렬화·첫 추론 지연을 루프 밖에서
    cam = D435i(width=cam_cfg["width"], height=cam_cfg["height"], fps=cam_cfg["fps"])
    cam.start()
    # 하네스/테스트의 FakeCam 에는 없다. 있으면 AE 측광 영역을 추적 bbox 로 따라가게 한다.
    set_ae_roi = getattr(cam, "set_exposure_roi", None) if cam_cfg.get("ae_roi_follow_track", True) else None
    intrinsics = cam.intrinsics
    print(f"[CAM] fx={intrinsics['fx']:.1f} fy={intrinsics['fy']:.1f} ppx={intrinsics['ppx']:.1f} ppy={intrinsics['ppy']:.1f}")

    meas_builder = MeasurementBuilder(intrinsics, depth_scale=cam.depth_scale)
    rel = ReliabilityEstimator()
    scheduler = PerceptionScheduler()
    tracker = LeaderTracker()
    ekf = ImmEkf()
    mission = MissionManager()
    leader_rx = open_leader_receiver()
    log = ExperimentLogger(CONFIG["logger"]["log_dir"]) if CONFIG["logger"]["enabled"] else None

    last_track = None
    prev_time = time.time()
    last_stat_print = 0.0
    last_setpoint_time = 0.0
    last_land_send = 0.0          # C2: LAND 재시도 타이머
    prev_fc_accepts = False       # GUIDED 진입 에지 검출용
    cam_fail_streak = fc_fail_streak = 0
    last_fused_rx_time = None     # 마지막으로 EKF 에 융합한 ESP32 패킷의 rx_time
    show_window = SHOW_WINDOW
    frame_idx = 0
    fps_counter, fps_t0, fps_display = 0, time.time(), 0.0
    prev_body_cmd = np.zeros(4)
    current_body_cmd = np.zeros(4)
    prev_rpy_for_comp = None      # 직전 프레임 팔로워 (roll, pitch, yaw)
    ff_fru = np.zeros(3)          # 리더 속도 피드포워드 (FRU, 저역통과 상태)
    v_leader_fru = None           # 리더 절대 속도 추정 (FRU). STAT 진단용으로 루프 밖에서도 참조

    print("=" * 90)
    print("[INFO] q/ESC 종료 | m: MARS-IMM on/off | v: MAVLink velocity 송신 on/off | l: LAND | h: HOLD")
    print(f"[INFO] SEND_MAVLINK_COMMANDS={SEND_MAVLINK_COMMANDS}  USE_LEADER_ESP32={USE_LEADER_ESP32} ({LEADER_TELEMETRY_KIND})")
    print("[INFO] 실제 비행 전 반드시 프로펠러 제거 상태에서 확인")
    print("=" * 90)

    try:
        while True:
            now = time.time()
            # prev_time 은 프레임 획득에 성공한 뒤에 갱신한다 — 카메라/FC 실패로 continue 한 반복의
            # 시간이 predict / range_coast 에서 사라지지 않게.
            dt = max(now - prev_time, 1e-4)

            # ---------------- Pixhawk 수신 ----------------
            try:
                drain_messages(master)
                fc_fail_streak = 0
            except Exception as exc:
                fc_fail_streak += 1
                print(f"[WARN] FC link: drain 실패 {fc_fail_streak}회: {type(exc).__name__}: {exc}")
                if fc_fail_streak >= FC_FAIL_LIMIT:
                    print(f"[ERR] FC 링크 연속 실패 {FC_FAIL_LIMIT}회 — 종료")
                    break
                continue
            vehicle_state = get_vehicle_state()

            # C2: FC 모드를 매 루프 읽는다. GUIDED/OFFBOARD 를 벗어났다 = 조종사가 탈환했다 → LAND 를 보내지 않는다.
            fc_mode = vehicle_state.get("mode", {}).get("name", "?")
            fc_armed = vehicle_state.get("mode", {}).get("armed", False)
            fc_accepts_setpoints = fc_mode in ("GUIDED", "OFFBOARD")

            # GUIDED 진입 = 조종사가 방금 자동에게 넘긴 순간. 그 전까지 FC 는 우리 명령을 버렸으므로 그동안 쌓인
            # 상태(수동 상승 중 FAILSAFE_LAND 로 래치된 미션, 포화된 평활 버퍼)를 들고 들어가면 안 된다.
            if fc_accepts_setpoints and not prev_fc_accepts:
                mission.reset()
                prev_body_cmd = np.zeros(4)
                ff_fru = np.zeros(3)
                last_land_send = 0.0
                print(f"[SYS] {fc_mode} 진입 — 미션/명령 리셋")
            prev_fc_accepts = fc_accepts_setpoints

            gps_fresh = is_fresh(vehicle_state.get("gps", {}), now, GPS_MAX_AGE_SEC)
            local_pos_fresh = is_fresh(vehicle_state.get("local_position", {}), now, LOCAL_POS_MAX_AGE_SEC)
            attitude_fresh = is_fresh(vehicle_state.get("attitude", {}), now, ATTITUDE_MAX_AGE_SEC)

            # ---------------- ESP32 수신 (표시·미션용은 매 프레임, 융합은 새 패킷만) ----------------
            leader_packet = leader_rx.read_latest() if leader_rx is not None else None
            leader_meas = build_leader_measurement_from_packet(
                packet=leader_packet, follower_vehicle_state=vehicle_state, now=now,
                max_age_sec=LEADER_MAX_AGE_SEC, leader_velocity_frame=LEADER_VELOCITY_FRAME)

            if now - last_stat_print >= 1.0:
                p_cv, p_ct = ekf.get_model_probs() if ekf.initialized else (0.0, 0.0)
                print(f"[STAT] {battery_text()} FPS={fps_display:.1f} MARS={'ON' if use_mars_imm else 'OFF'} "
                      f"CMD={'ON' if SEND_MAVLINK_COMMANDS else 'DRY'} FC={fc_mode}{'*' if fc_armed else ''} "
                      f"ESP={int(leader_meas.get('available', False))}:{leader_meas.get('reason', 'none')} "
                      f"CV={p_cv:.2f} CT={p_ct:.2f} coast={ekf.coast_time:.1f}s rcoast={ekf.range_coast_time:.1f}s "
                      f"mission={mission.state} "
                      f"vL={(float(np.linalg.norm(v_leader_fru)) if v_leader_fru is not None else float('nan')):.2f} "
                      f"ff={ff_fru[0]:+.2f} fresh=LP{int(local_pos_fresh)}/ATT{int(attitude_fresh)} {stream_rates_text(now)}")
                if SEND_MAVLINK_COMMANDS and not fc_accepts_setpoints:
                    print(f"[WARN] FC mode={fc_mode}: 송신 중단됨 (ArduPilot: GUIDED / PX4: OFFBOARD 필요)")
                last_stat_print = now

            # ---------------- 카메라 (예외·None 은 드롭, 연속 실패면 포기) ----------------
            try:
                color_image, depth_image = cam.get_frames()
            except Exception as exc:
                color_image, depth_image = None, None
                print(f"[WARN] 카메라 프레임 실패 {cam_fail_streak + 1}회: {type(exc).__name__}: {exc}")
            if color_image is None:          # 예외도, 컬러/깊이 결손(None)도 같은 드롭 — 둘 다 연속 실패로 센다
                cam_fail_streak += 1
                if cam_fail_streak >= CAM_FAIL_LIMIT:
                    print(f"[ERR] 카메라 연속 실패 {CAM_FAIL_LIMIT}회 — 종료")
                    break
                continue
            cam_fail_streak = 0

            prev_time = now
            frame_idx += 1
            H, W = color_image.shape[:2]
            fps_counter += 1
            if now - fps_t0 >= 1.0:
                fps_display = fps_counter / max(now - fps_t0, 1e-6)
                fps_counter, fps_t0 = 0, now

            # ---------------- IMM predict (팔로워 자세 변화만큼 상대상태를 역회전한 뒤) ----------------
            att = vehicle_state.get("attitude", {})
            if attitude_fresh and att.get("yaw") is not None:
                cur_rpy = (float(att.get("roll") or 0.0), float(att.get("pitch") or 0.0), float(att["yaw"]))
                if prev_rpy_for_comp is not None and ekf.initialized:
                    ekf.compensate_ego_rotation(ego_rotation_cam(prev_rpy_for_comp, cur_rpy))
                prev_rpy_for_comp = cur_rpy
            else:
                prev_rpy_for_comp = None
            if ekf.initialized:
                ekf.predict(dt)

            # ---------------- 스케줄러 → 검출/추적 ----------------
            if use_mars_imm:
                policy = scheduler.decide(ekf.get_state_dict(), color_image.shape, intrinsics, last_track)
            else:
                policy = {"run_detector": True, "use_full_frame": True, "roi": None, "detect_every": 1, "reason": "baseline_full_frame"}

            if policy["run_detector"]:
                track = tracker.update(detector.detect(color_image, roi=None if policy["use_full_frame"] else policy["roi"]))
            else:
                track = tracker.predict_only()
            last_track = track
            if set_ae_roi is not None:
                set_ae_roi(track["bbox"] if (track is not None and not track.get("is_lost", False)) else None, now)

            # ---------------- 측정 → 융합 ----------------
            rgbd_meas = meas_builder.build_rgbd(track, depth_image)
            bearing_meas = meas_builder.build_bearing(track) if track is not None else None
            r_vis = rel.vision_reliability(rgbd_meas or bearing_meas or track)
            r_depth = rel.depth_reliability(rgbd_meas)
            update_used, gate_d2 = fuse_vision(ekf, rel, rgbd_meas, bearing_meas, r_vis, r_depth, use_mars_imm)

            # read_latest() 는 새 패킷이 없으면 같은 패킷을 다시 돌려준다. 같은 관측을 매 프레임 독립 측정처럼
            # 융합하면 안 되므로 새 패킷(rx_time 변경)일 때만 융합한다.
            esp = ESP_IDLE
            esp_rx_time = leader_meas.get("rx_time", None)
            if USE_LEADER_ESP32 and leader_meas.get("available", False) and esp_rx_time is not None \
                    and esp_rx_time != last_fused_rx_time:
                last_fused_rx_time = esp_rx_time
                esp = fuse_esp32(ekf, rel, leader_meas)

            if (track is None or track.get("is_lost", False)) and ekf.initialized:
                ekf.on_lost(dt)

            # ---------------- 추정 상태 → 미션 ----------------
            x_est, P_est = ekf.get_state()
            pos_cov_trace = float(np.trace(P_est[:3, :3])) if ekf.initialized else 999.0
            mu = ekf.get_model_probs() if ekf.initialized else np.array([0.0, 0.0])
            rel_fru = camera_xyz_to_fru(x_est[:3])
            rel_vel_fru = camera_xyz_to_fru(x_est[3:6])

            follower_alt = get_follower_altitude_m(vehicle_state) if local_pos_fresh else None
            # 착륙 판정용 선두 고도는 AGL 근사여야 한다 (ESP32 alt 는 절대고도라 landing_z_thresh 와 비교 불가)
            # → 팔로워 AGL(LOCAL_POSITION_NED) + 상대고도.
            leader_alt_est = float(follower_alt + rel_fru[2]) if (follower_alt is not None and ekf.initialized and ekf.is_reliable()) else None
            esp_visible = bool(leader_meas.get("available", False))
            leader_vel_world = np.asarray(leader_meas["leader_vel_enu"], dtype=float) if esp_visible else None

            # 미션의 "리더가 보인다" = 거리를 아는가. bbox 유무나 is_reliable()(bearing-only 로도 참)은 거리 관측을
            # 보장하지 않아 깊이가 죽어도 소실 판정이 안 나기 때문이다. RGB-D 와 ESP32 위치만 range_coast 를 되돌린다.
            leader_visible_for_mission = bool(ekf.has_range_fix())
            # 추종 거리는 거리의 출처로: 카메라 깊이가 살아 있으면 3m, ESP32 GPS 뿐이면 오차 여유를 둔 8m.
            vision_range_ok = bool(ekf.has_vision_range_fix())
            target_distance_m = TARGET_DISTANCE_M if vision_range_ok else TARGET_DISTANCE_GPS_ONLY_M

            # 리더 절대 속도(FRU) = FC 자기 속도 + EKF 상대 속도. 미션(출발/정지/착륙 판단)과 피드포워드가 같이 쓴다.
            # 상대 속도만 보면 후미가 선두 속도를 맞추는 순간 0 이 되어 '선두 정지' 로 오판한다. 자기 속도·자세가
            # 신선하고 EKF 가 신뢰할 수 있을 때만 만들고, 없으면 미션은 상대 속도로 폴백한다.
            v_leader_fru = None
            if ekf.initialized and ekf.is_reliable() and local_pos_fresh and attitude_fresh:
                v_f = follower_velocity_fru(vehicle_state)
                if v_f is not None:
                    v_leader_fru = v_f + rel_vel_fru

            mission_state, mission_policy = mission.update(
                now=now, leader_visible=leader_visible_for_mission,
                rel_est=rel_fru if ekf.initialized else None, rel_vel_est=rel_vel_fru if ekf.initialized else None,
                leader_alt=leader_alt_est, leader_vel_world=leader_vel_world, pos_cov_trace=pos_cov_trace,
                leader_vel_body=v_leader_fru)

            # ---------------- 명령 ----------------
            # 리더 속도 피드포워드: 거리를 아는 추종 상태에서만. 아니면 0 으로 감쇠.
            ff_fru = leader_velocity_ff(
                ff_fru, v_leader_fru if (mission_policy["allow_follow"] and ekf.has_range_fix()) else None, dt)

            desired_body_cmd = np.zeros(4)
            if mission_policy["land"]:
                # C2: 모드 게이트가 곧 latch 다 — LAND 가 먹으면 FC 가 GUIDED 를 벗어나 이 분기가 더 실행되지
                # 않는다. 여전히 여기 있다는 건 명령이 먹지 않았다는 뜻이라 그때만 LAND_RETRY_SEC 간격으로 재시도
                # (조종사 탈환 시엔 fc_accepts_setpoints=False 라 아예 보내지 않는다).
                if SEND_MAVLINK_COMMANDS and fc_accepts_setpoints and now - last_land_send >= LAND_RETRY_SEC:
                    try:
                        send_land(master)
                    except Exception as exc:
                        print(f"[WARN] FC link: LAND 송신 실패: {type(exc).__name__}: {exc}")
                    last_land_send = now
                    last_setpoint_time = now
            elif mission_policy["allow_follow"] and ekf.initialized:
                desired_body_cmd = compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, pos_cov_trace, target_distance_m, ff_fru)

            # 수직 축은 리더 상대 고도를 따라간다 — 트래커가 지면의 무언가를 물면 계속 하강한다. AGL 바닥.
            if follower_alt is not None and follower_alt < MIN_AGL_M and desired_body_cmd[2] > 0.0:
                desired_body_cmd[2] = 0.0

            current_body_cmd = smooth_velocity_cmd(prev_body_cmd, desired_body_cmd, alpha=0.28, dt=dt)
            prev_body_cmd = current_body_cmd.copy()

            # 속도 setpoint 에는 모드 게이트를 걸지 않는다: GUIDED/OFFBOARD 가 아니면 FC 가 조용히 버리고, PX4 는
            # OFFBOARD 진입 전에 이 스트림이 먼저 흐르고 있어야 한다. 조종사를 뺏는 건 모드 변경(LAND)이며 그쪽만 막는다.
            if not mission_policy["land"]:
                last_land_send = 0.0
                if now - last_setpoint_time >= SETPOINT_PERIOD_SEC:
                    if SEND_MAVLINK_COMMANDS:
                        try:
                            send_body_velocity(master, *current_body_cmd[:3], yaw_rate=current_body_cmd[3])
                        except Exception as exc:
                            print(f"[WARN] FC link: setpoint 송신 실패: {type(exc).__name__}: {exc}")
                    # 위상 고정: 다음 송신 시각을 '지금'이 아니라 '직전 예정 시각 + 주기'로 잡는다. '지금'으로
                    # 잡으면 프레임 경계에 맞춰 늦어져 30fps 에서 3×(1/30)<0.1 → 4프레임마다(7.5Hz)만 나갔다.
                    # 오래 멈췄다 돌아오면(카메라 스톨) 한 번에 따라잡지 않고 다음 주기부터 다시 센다.
                    last_setpoint_time = max(last_setpoint_time + SETPOINT_PERIOD_SEC, now - SETPOINT_PERIOD_SEC)

            # ---------------- 화면 (DISPLAY_EVERY 프레임마다) ----------------
            if show_window and frame_idx % DISPLAY_EVERY == 0:
                raw_depth_m = rgbd_meas.get("depth_m") if rgbd_meas else None
                ekf_depth_m = float(x_est[2]) if ekf.initialized and ekf.is_reliable() else None
                if track is not None and not track.get("is_lost", False):
                    x1, y1, x2, y2 = track["bbox"]
                    cx_t, cy_t = bbox_center(track["bbox"])
                    col = depth_color(ekf_depth_m if ekf_depth_m is not None else raw_depth_m)
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), col, 2)
                    cv2.circle(color_image, (int(cx_t), int(cy_t)), 5, (0, 0, 255), -1)
                    raw_s = f"{raw_depth_m:.2f}m" if raw_depth_m is not None else "N/A"
                    ekf_s = f"{ekf_depth_m:.2f}m" if ekf_depth_m is not None else "N/A"
                    put_text(color_image, f"raw={raw_s} est={ekf_s} {update_used}", (x1, max(22, y1 - 8)), col, scale=0.48)
                if not policy.get("use_full_frame", True) and policy.get("roi") is not None:
                    rx1, ry1, rx2, ry2 = policy["roi"]
                    cv2.rectangle(color_image, (rx1, ry1), (rx2, ry2), (255, 0, 255), 1)
                cv2.line(color_image, (W // 2, 0), (W // 2, H), (0, 255, 255), 1)
                draw_hud(color_image, [
                    (f"MARS:{'ON' if use_mars_imm else 'OFF'} CMD:{'ON' if SEND_MAVLINK_COMMANDS else 'DRY'}", (255, 255, 0), 0.60),
                    (battery_text(), (0, 255, 255), 0.60),
                    (f"FPS={fps_display:.1f} policy={policy.get('reason')}", (200, 200, 200), 0.50),
                    (f"mission={mission_state} mode={mission_policy['mode']}", (100, 255, 255), 0.50),
                    (f"rV={r_vis:.2f} rD={r_depth:.2f} gate={gate_d2 if gate_d2 is not None else -1:.1f}", (180, 180, 255), 0.50),
                    (f"ESP={int(esp_visible)} {leader_meas.get('reason', 'none')} upd={esp['esp_update_used']}", (180, 220, 255), 0.48),
                    (f"rel F/R/U=({rel_fru[0]:+.2f},{rel_fru[1]:+.2f},{rel_fru[2]:+.2f}) cov={pos_cov_trace:.2f} "
                     f"tgt={target_distance_m:.1f}m{'' if vision_range_ok else '(GPS)'}", (220, 220, 220), 0.48),
                    (f"cmd BODY_NED vx={current_body_cmd[0]:+.2f} vy={current_body_cmd[1]:+.2f} "
                     f"vz={current_body_cmd[2]:+.2f} yr={current_body_cmd[3]:+.2f} ffF={ff_fru[0]:+.2f}", (100, 255, 100), 0.48),
                    (f"fresh GPS/LP/ATT={int(gps_fresh)}/{int(local_pos_fresh)}/{int(attitude_fresh)}", (180, 180, 255), 0.48),
                ])
                if ekf.initialized:
                    draw_model_bar(color_image, ekf.get_model_probs())
                try:
                    cv2.imshow("MARS-IMM Drone Follow", color_image)
                    key = cv2.waitKey(1) & 0xFF
                except Exception as exc:
                    # DISPLAY 가 없거나 ssh -X 가 끊기면 imshow 가 예외를 던진다. 창을 포기하고 헤드리스로 계속.
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

            # ---------------- 로그 ----------------
            if log is not None:
                log.log(build_log_row(dict(ff_fru=ff_fru, 
                    now=now, dt=dt, fps_display=fps_display, use_mars_imm=use_mars_imm, policy=policy,
                    mission_state=mission_state, mission_policy=mission_policy, track=track, rgbd_meas=rgbd_meas,
                    r_vis=r_vis, r_depth=r_depth, gate_d2=gate_d2, update_used=update_used, leader_meas=leader_meas,
                    esp=esp, x_est=x_est, pos_cov_trace=pos_cov_trace, mu=mu, ekf=ekf, rel_fru=rel_fru,
                    rel_vel_fru=rel_vel_fru, vehicle_state=vehicle_state, gps_fresh=gps_fresh,
                    local_pos_fresh=local_pos_fresh, attitude_fresh=attitude_fresh, current_body_cmd=current_body_cmd,
                    target_distance_m=target_distance_m, vision_range_ok=vision_range_ok)))

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
        # FC 소켓도 닫는다. 안 닫으면 udpin 포트를 계속 쥐고 있어 같은 프로세스에서 다시 연결할 때(SITL 하네스가
        # 시나리오마다 main 을 재실행) 새 소켓이 패킷을 못 받아 heartbeat 를 영원히 기다린다.
        for closer in ((leader_rx.close if leader_rx is not None else None), cam.stop, getattr(master, "close", None)):
            try:
                if closer:
                    closer()
            except Exception:
                pass
        if log is not None:
            log.close()
        cv2.destroyAllWindows()
        print("[SYS] done")


if __name__ == "__main__":
    main()
