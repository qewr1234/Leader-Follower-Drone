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
import signal
import time

import cv2
import numpy as np
from pymavlink import mavutil

import mavlink_io
from camera import D435i
from config import CONFIG, validate_config
from detector import YoloDetector
from imm_ekf import ImmEkf
from leader_telemetry import (LeaderTelemetryReceiver, apply_leader_velocity_update_to_imm,
                              build_leader_measurement_from_packet,
                              fru_to_camera_xyz)  # camera_xyz_to_fru 의 역변환. analysis 가 main.fru_to_camera_xyz 로 참조
from logger import ExperimentLogger
from mavlink_io import (HEARTBEAT_MAX_AGE_SEC, battery_text, connect_fc, drain_messages, get_vehicle_state, reconnect_fc,
                        request_data_streams, send_heartbeat, stream_rates_text)
from measurement import MeasurementBuilder
from mission_manager import MissionManager
from reliability import ReliabilityEstimator
from scheduler import CHI2_2D_99, PerceptionScheduler
from tracker import LeaderTracker
from utils_geometry import bbox_center, camera_to_pixel, clamp

# ============================================================
# 실행 설정
# ============================================================

SHOW_WINDOW = os.environ.get("MARS_SHOW_WINDOW", "1") != "0"
# 화면 출력(그리기+imshow+waitKey)은 비-YOLO 비용 중 가장 크다(Jetson 4~8ms). 2프레임에 한 번(15Hz)만
# 그린다. 키 입력(q/v/h/l)도 그 프레임에서만 읽히므로 최대 1프레임 늦게 반응한다.
DISPLAY_EVERY = 2

# 처음에는 반드시 False. True 로 바꾸기 전: 프로펠러 제거 → Pixhawk 모드 확인 → 화면 명령값 확인 → 저속.
SEND_MAVLINK_COMMANDS = False

USE_MARS_IMM_DEFAULT = True   # False = 연구용 baseline(고정 R, 게이트·스케줄러 우회). 시작 시에만 정하고 비행 중에는 바꿀 수 없다. 어떤 시험도 False 경로를 검증하지 않았다.
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

# 목표 추종 거리. 정상상태 이격 = TARGET + (v − KFF·(v−DB) + KV·v)/Kp (0.3 m/s 에서 +0.63 m,
# compute_velocity_cmd_from_estimate 주석). 이 값과 config 의 depth_max_m 간격이 깊이창 안에서 추종 가능한
# 리더 속도의 상한을 정하지만(C4), 실질 상한은 그보다 MAX_VX 가 먼저 건다.
TARGET_DISTANCE_M = 3.0
# 카메라 깊이 없이 ESP32 GPS 상대위치만으로 거리를 알 때의 이격. GPS 상대오차는 m 단위라 3m 는 오차보다 작다.
# 8 m 는 D435i 의 실용 깊이 범위(≈ 6 m, 그 밖은 오차가 거리 제곱으로 커진다 — reliability 의 거리 배율 ×7 이상) **밖**이다: 이 거리에서 비전이
# 바로 이어받는 것이 아니라, 리더가 다시 6 m 안으로 들어올 때 깊이가 살아나 3 m 로 램프한다. 더 줄이면 GPS 오차(2~5 m)가 최소 이격 2 m 를
# 침범하므로 8 m 를 유지한다(이전 감사 #46 은 README 의 '깊이창 안이라 바로 이어받는다' 는 근거를 지적한 것 — 문구를 고쳤다).
TARGET_DISTANCE_GPS_ONLY_M = 8.0

# BODY_NED 속도 제한 (x=forward, y=right, z=down). 첫 실비행은 더 낮게(0.15/0.08/0.10) 권장.
# MAX_VZ 0.12 → 0.25 (2026-10-01): 수직 FOV 가 ±21°(3 m 에서 ±1.2 m)라 리더의 0.5 m/s 상승·하강을 0.12 로는 못 따라가
# 2 s 안에 시야를 잃고 소실 착륙으로 갔다. 리더 수직속도가 MAX_VZ 를 넘으면 여전히 소실되므로 운용 제한(README)에 적는다.
MAX_VX = 0.35
MAX_VY = 0.22
MAX_VZ = 0.25

# 전후 Kp 0.22 → 0.30 (2026-09-19): 추정기가 자기 속도를 예측 입력으로 받으면서 이득여유가 14 → 27 dB 로 커져,
# 시간간격 정책(KV_SELF)이 늘리는 정상상태 이격을 Kp 로 되사는 여유가 생겼다 (docs/STABILITY_MARGINS.md 7절).
KP_FORWARD, KD_FORWARD = 0.30, 0.05
KP_RIGHT, KD_RIGHT = 0.28, 0.04
KP_UP, KD_UP = 0.18, 0.03
# yaw: 선두를 카메라 시야(FOV ~69도) 중앙에 유지. bearing(rad) 오차 → yaw_rate(rad/s)
KP_YAW = 0.8
MAX_YAW_RATE = 0.35

UNCERTAINTY_SLOWDOWN_TRACE = CONFIG["controller"].get("uncertainty_slowdown_trace", 4.0)
# 리더 속도 피드포워드 (config controller.leader_vel_ff_*)
KFF_LEADER_VEL = float(CONFIG["controller"].get("leader_vel_ff_gain", 0.8))
FF_TAU_SEC = float(CONFIG["controller"].get("leader_vel_ff_tau_sec", 0.1))
# 자기 속도 감쇠 (시간간격 정책). cmd 에 −KV_SELF·v_self 를 더한다 = 전후축에서는 목표 이격이 3.0 + (KV/Kp)·v 로
# 속도에 비례해 벌어지는 constant-time-gap 정책과 같다. 등간격 정책은 선행 기체 정보만으로는 스트링 안정이
# 안 되고(|Γ| 피크 1.05~1.14), 시간간격 h = KV/Kp = 1.0 s 가 이를 FC 지연 0.3~0.8 s 전 범위에서 1.00 으로 내린다
# (docs/STABILITY_MARGINS.md 7절, 고전 규칙 h ≥ 2τ).
KV_SELF = float(CONFIG["controller"].get("self_vel_damping", 0.3))
# 상하축은 따로(0.15): 0.3 이면 수직 뒤처짐 2.78 m/(m/s), 0 이면 |Γ| 1.03 (config 주석의 분석 표).
KV_SELF_UP = float(CONFIG["controller"].get("self_vel_damping_up", KV_SELF))
FF_DEADBAND_MPS = float(CONFIG["controller"].get("leader_vel_ff_deadband_mps", 0.05))
# 거리 코스트 중 추종 항 페이드 시작 시각 (RANGE_COAST_MAX_SEC 에서 0). coast_command_scale 참조.
COAST_FADE_START_SEC = float(CONFIG["controller"].get("coast_fade_start_sec", 0.3))
RANGE_COAST_MAX_SEC = float(CONFIG["imm"].get("range_coast_max_sec", 2.0))
# EKF 초기화 확인: 연속 INIT_CONFIRM_FRAMES 프레임의 RGB-D 측정이 서로 INIT_CONSISTENCY_M 안에 있어야 시작한다 (InitConfirm).
INIT_CONFIRM_FRAMES = int(CONFIG["imm"].get("init_confirm_frames", 3))
INIT_CONSISTENCY_M = float(CONFIG["imm"].get("init_consistency_m", 0.75))

# 최소 이격: 이 거리 아래로는 접근 성분을 0 으로 자르고 침범량에 비례해 물러난다 (compute_velocity_cmd_from_estimate).
MIN_SEPARATION_M = float(CONFIG["controller"].get("min_separation_m", 2.0))
MIN_SEPARATION_KP = float(CONFIG["controller"].get("min_separation_kp", 0.6))
# 측면 회피: 후퇴가 MAX_VX 에 포화된 뒤의 마지막 수단.
EVADE_RADIUS_M = float(CONFIG["controller"].get("evade_radius_m", 1.5))
EVADE_SPEED_MPS = float(CONFIG["controller"].get("evade_speed_mps", 0.22))
EVADE_RAMP_M = max(float(CONFIG["controller"].get("evade_ramp_m", 0.3)), 1e-6)
EVADE_SIDE_DEADBAND_M = float(CONFIG["controller"].get("evade_side_deadband_m", 0.30))

# 트랙 재획득 힌트 반경에 더하는 여유 [px] 와, 3-D 게이트가 연속으로 거부하면 트랙을 버리는 횟수 (reacquire_hint, LeaderTracker).
REACQ_MARGIN_PX = int(CONFIG["scheduler"].get("reacquire_margin_px", 60))
GATE_REJECT_DROP_FRAMES = int(CONFIG["scheduler"].get("gate_reject_drop_frames", 3))

# 회피 방향 래치. 0=미결정, +1=오른쪽, -1=왼쪽. 반경을 벗어나면 풀린다.
_evade_side = 0


def reset_evade_side():
    """GUIDED 진입처럼 상태를 새로 시작할 때 회피 방향 래치를 푼다."""
    global _evade_side
    _evade_side = 0

SETPOINT_PERIOD_SEC = 0.10
# C2: LAND 가 먹지 않았을 때만 이 간격으로 재시도. 100ms 연타는 조종사 탈환을 덮어쓴다.
LAND_RETRY_SEC = 2.0
CAM_FAIL_LIMIT = 30      # 카메라 연속 실패 한계(실패 프레임에도 미션·setpoint 는 계속 돈다 — 소실로 취급). 넘으면 LOST_ACTION 을 한 번 보내고 종료.
FC_FAIL_SEC = 15.0       # FC 링크(drain) 예외가 이 시간 이어지면 포기 (예전: 횟수 30회 = 1 ms). 그 사이 FC_RECONNECT_SEC 마다 재연결 시도.
FC_RECONNECT_SEC = 1.0
# 리더 소실 10 s 뒤·카메라 사망 종료 시의 행동 (config controller.lost_action: land | rtl | hold).
LOST_ACTION = str(CONFIG["controller"].get("lost_action", "land")).lower()
# 추종 추정 거리 상한(AP_Follow FOLL_DIST_MAX 상당): 이보다 멀면 거리 측정으로 치지 않아 먼 오검출을 쫓지 않는다.
MAX_FOLLOW_DIST_M = float(CONFIG["controller"].get("max_follow_dist_m", 15.0))
# 상승 천장(home 기준). MIN_AGL_M 의 거울상. FC FENCE_ALT_MAX 가 1차 방어, 이것은 컴패니언 쪽 2차.
MAX_ALT_M = float(CONFIG["controller"].get("max_alt_m", 30.0))
# 목표 이격 3 m ↔ 8 m 전환 램프 [m/s]. 계단으로 바꾸면 P 항이 Kp·5 m 만큼 뛰어 ±MAX_VX 로 포화된다.
TARGET_DISTANCE_RAMP_MPS = float(CONFIG["controller"].get("target_distance_ramp_mps", 0.3))
# 루프 워치독: 한 프레임이 이보다 오래 걸리면 경고, LOOP_RESET_SEC(GUID_TIMEOUT) 를 넘으면 FC 는 이미 정지했으므로 명령 상태를
# 0 에서 다시 시작한다 — 멈춰 있던 동안의 평활 버퍼·피드포워드로 갑자기 재출발하지 않게. 단일 스레드라 멈춘 동안 할 수 있는 일은 없고,
# 복귀 뒤의 처리만 정한다(이전 감사 '루프 시간 워치독 부재').
LOOP_STALL_SEC = 1.0
LOOP_RESET_SEC = 3.0
# AE 측광 ROI 를 전체 프레임으로 되돌리는 연속 미검출 수 (1프레임 미스마다 되돌리면 1 Hz 로 측광이 왕복한다).
AE_ROI_RELEASE_LOST = int(CONFIG["camera"].get("ae_roi_release_lost_frames", 5))
MAV_STATE_CRITICAL = 5
# 고도 바닥. 고도 출처 우선순위(get_follower_altitude_m): 하향 거리계(RANGEFINDER/DISTANCE_SENSOR, 진짜 AGL) → LOCAL_POSITION_NED −z →
# GLOBAL_POSITION_INT relative_alt (뒤 둘은 home 기준 — 이륙점이 평지면 AGL 근사). 모르면 하강 금지(fail-closed).
MIN_AGL_M = 1.5
RANGEFINDER_MAX_AGE_SEC = 0.5
# 데이터 스트림 재요청: heartbeat 는 오는데 LOCAL_POSITION_NED 가 이 시간 넘게 안 오면 request_data_streams 를 다시 보낸다 (연결 시 한 번만
# 요청하던 것 — FC 재부팅·라우터 재연결 뒤 스트림이 조용히 사라졌다, 이전 감사 #54). 5 s 간격.
STREAM_REREQUEST_SEC = 5.0
MAV_CMD_DO_SET_MODE = 176
MAV_RESULT_ACCEPTED = 0

# ATTITUDE 불연속 판정 [rad]. FC EKF 가 비행 중 yaw 를 재정렬하면(나침반 불일치, GSF 리셋) 기체는 돌지 않았는데 보고 yaw 만
# 수십 도 뛴다. 그것을 '카메라가 돌았다' 로 보상하면 리더 추정이 3 m 에서 Δψ·3 m 만큼 옆으로 튀어 1.4~2.9 s 전부거부·측면
# 포화가 난다(2026-10-01 감사 5번). 한 ATTITUDE 샘플 사이의 물리적 yaw 변화는 yawspeed·Δt 로 예측되므로, 그 예측과
# 0.2 rad(11°) 넘게 다르면 센서 불연속으로 보고 그 프레임의 보상을 건너뛴다. yawspeed 가 없으면 크기만으로 0.3 rad.
ATT_YAW_JUMP_RAD = 0.2
ATT_YAW_JUMP_ABS_RAD = 0.3

# GPS_RAW_INT 신선도 — 표시·로그 전용. ESP32 융합의 팔로워 GPS 입력은 이 값으로 게이트하지 않는다(leader_telemetry 참조).
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


def rot_body_to_ned(roll, pitch, yaw):
    """MAVLink ATTITUDE(ZYX 오일러) → 기체→NED 회전행렬 Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


# 카메라 프레임(x=우, y=하, z=전) ← 카메라 마운트에 정렬된 기체 FRD(x=전, y=우, z=하): 축 순열.
_CAM_PERM = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
# 카메라 마운트 자세(config camera.mount_*_deg): 기체 FRD 벡터를 마운트 정렬 프레임으로 돌리는 R_mount^T 를 순열 앞에 둔다.
# 마운트가 0 이면 _CAM_FROM_BODY 는 순열 그대로다. 아래로 10° 숙여 단 카메라(mount_pitch −10°)는 전방 3 m 의 리더를 카메라에서
# 0.52 m '위' 로 보는데, 이 회전이 없으면 그만큼 상승 명령이 나간다(이전 감사 #42).
_CAM_MOUNT_RPY = tuple(math.radians(float(CONFIG["camera"].get(k, 0.0))) for k in ("mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg"))
_R_BODY_FROM_MOUNT = rot_body_to_ned(*_CAM_MOUNT_RPY)        # 마운트 프레임 → 기체 FRD (같은 ZYX 합성)
_CAM_FROM_BODY = _CAM_PERM @ _R_BODY_FROM_MOUNT.T              # 기체 FRD → 카메라
# 카메라 레버암 (기체 FRD, m): 기체 기준점 → 카메라 광학 중심. 상대 위치에만 더한다 (config camera.mount_offset_frd_m).
_CAM_OFFSET_FRD = np.asarray(CONFIG["camera"].get("mount_offset_frd_m", [0.0, 0.0, 0.0]), dtype=float)[:3]


def level_from_body(roll, pitch):
    """기체 FRD → 수평(기수 정렬, roll/pitch 를 편) 프레임: Ry(pitch)·Rx(roll). FC 의 BODY_NED 속도는 yaw 만 돌리고 z 는 지구 하방이라,
    제어 입력은 이 수평 프레임에 있어야 한다(이전 감사 #2·#22: 기울어진 카메라 프레임을 그대로 쓰면 pitch 가 수직 명령으로 샌다)."""
    return rot_body_to_ned(roll, pitch, 0.0)


def camera_to_level_fru(x_cam, roll=0.0, pitch=0.0, position=False):
    """카메라 [right, down, forward] → 수평 FRU [front, right, up] (마운트와 기체 roll/pitch 를 편 것). roll=pitch=0·마운트 0 이면
    camera_xyz_to_fru 와 같다. position=True 면 레버암(_CAM_OFFSET_FRD, 기체 기준점 → 카메라)을 더해 '기체 기준점에서 본' 상대 위치가
    된다 — 속도에는 쓰지 않는다(ω×r 무시)."""
    v_frd = _CAM_FROM_BODY.T @ np.asarray(x_cam, dtype=float)[:3]
    if position:
        v_frd = v_frd + _CAM_OFFSET_FRD
    v_frd = level_from_body(roll, pitch) @ v_frd
    return np.array([v_frd[0], v_frd[1], -v_frd[2]], dtype=float)


def level_fru_to_camera_xyz(v_fru, roll=0.0, pitch=0.0):
    """수평 FRU → 카메라 [right, down, forward]. camera_to_level_fru 의 역변환 (EKF 의 자기 속도 입력은 카메라 프레임이어야 한다)."""
    v = np.asarray(v_fru, dtype=float)[:3]
    v_frd_level = np.array([v[0], v[1], -v[2]], dtype=float)
    return _CAM_FROM_BODY @ (level_from_body(roll, pitch).T @ v_frd_level)


def ego_rotation_cam(prev_rpy, cur_rpy):
    """팔로워 자세가 prev→cur 로 바뀌었을 때, 이전 카메라 프레임의 벡터를 현재 카메라 프레임으로 옮기는 3x3.

    리더가 월드에 고정돼 있어도 기체가 돌면 카메라 안에서 움직여 보인다 — yaw 만 아니라 돌풍에 의한
    roll/pitch 도 마찬가지다(10° pitch ≈ 640px 화면에서 68px). p_b2 = R2ᵀ·R1·p_b1 을 카메라 프레임으로 옮긴 것.
    """
    d_rb = rot_body_to_ned(*cur_rpy).T @ rot_body_to_ned(*prev_rpy)
    return _CAM_FROM_BODY @ d_rb @ _CAM_FROM_BODY.T


def fru_to_body_ned_velocity(v_fru):
    """수평 FRU [forward, right, up] → MAV_FRAME_BODY_NED 속도 [forward, right, down]. 이름과 달리 BODY_NED 는 기체 FRD 가 아니다 —
    ArduCopter/PX4 는 x·y 를 기수(yaw)만 돌린 수평 프레임으로, z 를 지구 하방(+)으로 해석한다. 그래서 입력이 '수평 FRU' 여야 하고
    변환은 z 부호 반전뿐이다."""
    v = np.asarray(v_fru, dtype=float)
    return np.array([v[0], v[1], -v[2]], dtype=float)


def get_follower_altitude_m(vehicle_state, now=None):
    """(고도 m, 출처) — 출처 우선순위: 하향 거리계(진짜 AGL) → LOCAL_POSITION_NED −z → GLOBAL_POSITION_INT relative_alt (둘 다 home 기준).
    now 를 주면 각 출처의 신선도(거리계 0.5 s, 위치 0.4 s, 전역 0.7 s)를 본다. 아무것도 없으면 (None, None) — enforce_agl_floor 가 하강을 막는다.
    PX4 기본 포트 프로파일은 LOCAL_POSITION_NED 를 안 보내지만 GLOBAL_POSITION_INT 는 보내므로(이전 감사 #62), 그 경우에도 바닥·착륙 판정이
    산다. 거리계는 ArduPilot RNGFND 가 있을 때만 온다."""
    rf = vehicle_state.get("rangefinder", {})
    if rf.get("distance") is not None and (now is None or is_fresh(rf, now, RANGEFINDER_MAX_AGE_SEC)):
        return float(rf["distance"]), "rangefinder"
    lp = vehicle_state.get("local_position", {})
    if lp.get("z") is not None and (now is None or is_fresh(lp, now, LOCAL_POS_MAX_AGE_SEC)):
        return -float(lp["z"]), "local_position"
    gp = vehicle_state.get("global_position", {})
    if gp.get("relative_alt") is not None and (now is None or is_fresh(gp, now, GPS_MAX_AGE_SEC)):
        return float(gp["relative_alt"]) / 1000.0, "global_position"
    return None, None


def enforce_agl_floor(cmd_body, follower_alt):
    """고도 바닥과 천장. 바닥: follower_alt 가 MIN_AGL_M 아래이거나 **모르면**(LOCAL_POSITION_NED 가 낡거나 없음) 하강 성분
    (BODY_NED z > 0)을 0 으로. 천장: MAX_ALT_M 위에서는 상승 성분(z < 0)을 0 으로 (모르면 상승은 막지 않는다 — 수직 이동을 전부
    막으면 안 되고, 상승 쪽 1차 방어는 FC FENCE_ALT_MAX 다).

    예전에는 `follower_alt is not None and ...` 이라 고도 스트림이 끊기면 바닥이 조용히 사라졌다(fail-open). 트래커가 지면의
    무언가를 물고 내려가는 상황이 바로 고도 스트림까지 의심스러운 상황이라, 모르면 막는 쪽이 맞다. 대가: LOCAL_POSITION_NED 를
    안 주는 FC 설정(PX4 기본 포트)에서는 하강 추종이 안 된다 — 그 경우 상승·수평은 되고 하강만 막히며 STAT 의 fresh=LP0 로 보인다.
    """
    cmd = np.asarray(cmd_body, dtype=float).copy()
    if (follower_alt is None or float(follower_alt) < MIN_AGL_M) and cmd[2] > 0.0:
        cmd[2] = 0.0
    if follower_alt is not None and float(follower_alt) > MAX_ALT_M and cmd[2] < 0.0:
        cmd[2] = 0.0
    return cmd


def attitude_jump(prev_att, cur_att):
    """두 ATTITUDE 샘플 사이의 yaw 변화가 물리적으로 설명되지 않으면 True (FC yaw 재정렬 등 센서 불연속).

    prev_att/cur_att: mavlink_io 의 attitude dict(yaw, yawspeed, timestamp). yawspeed 가 양쪽에 있으면 평균 각속도·Δt 로
    예상 변화를 만들어 ATT_YAW_JUMP_RAD 와 비교하고, 없으면 변화 크기만 ATT_YAW_JUMP_ABS_RAD 와 비교한다.
    """
    try:
        dyaw = (float(cur_att["yaw"]) - float(prev_att["yaw"]) + math.pi) % (2 * math.pi) - math.pi
    except (KeyError, TypeError, ValueError):
        return False
    if not math.isfinite(dyaw):
        return True
    w0, w1 = prev_att.get("yawspeed"), cur_att.get("yawspeed")
    t0, t1 = prev_att.get("timestamp"), cur_att.get("timestamp")
    if None not in (w0, w1, t0, t1) and all(math.isfinite(float(v)) for v in (w0, w1, t0, t1)):
        dt_att = max(float(t1) - float(t0), 0.0)
        expected = 0.5 * (float(w0) + float(w1)) * dt_att
        return abs(dyaw - expected) > ATT_YAW_JUMP_RAD
    return abs(dyaw) > ATT_YAW_JUMP_ABS_RAD


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
    """BODY_NED 속도 setpoint. vx forward+, vy right+, vz down+ [m/s], yaw_rate 우회전+ [rad/s].

    비유한 값(NaN/inf)은 여기서 0 으로 바꾼다 — 상류 어디서 새든 와이어에 닿기 전 마지막 방어다. 이 검사가
    없으면 NaN 추정이 clamp 를 지나 전 축 최대 속도로 나갔다(2026-09-24). 정상 경로에서는 절대 걸리지 않는다.
    """
    if not all(math.isfinite(float(v)) for v in (vx, vy, vz, yaw_rate)):
        print(f"[ERR] 비유한 속도 명령 ({vx}, {vy}, {vz}, {yaw_rate}) — HOLD(0) 로 대체")
        vx = vy = vz = yaw_rate = 0.0
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,     # time_boot_ms: 규격은 '부팅 후 ms' — 단조 시계가 그에 가장 가깝다
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
    if set_mode(master, "LAND"):                   # pymavlink mode_mapping 키는 ArduPilot/PX4 모두 "LAND"
        return
    print("[FC] send MAV_CMD_NAV_LAND")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def send_failsafe(master, action):
    """미션의 failsafe 정책(None | "LAND" | "RTL")을 FC 에 보낸다. None(hold)이면 아무것도 보내지 않는다(0 속도 setpoint 가 흐른다)."""
    if action == "LAND":
        send_land(master)
    elif action == "RTL":
        if not set_mode(master, "RTL"):
            print("[FC] RTL 모드 없음 — LAND 로 대체")
            send_land(master)


_SIGNAL_INSTALLED = False


def install_signal_handlers():
    """SIGTERM/SIGHUP 을 KeyboardInterrupt 로 바꿔 main 의 finally(마지막 HOLD·카메라/포트 닫기·CSV)가 실행되게 한다
    (이전 감사 #51: SIGTERM 은 finally 를 건너뛰었다). 메인 스레드가 아니거나 플랫폼이 거부하면 조용히 건너뛴다."""
    global _SIGNAL_INSTALLED

    def _raise(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for name in ("SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is None:
            continue
        try:
            signal.signal(sig, _raise)
            _SIGNAL_INSTALLED = True
        except (ValueError, OSError):
            pass
    return _SIGNAL_INSTALLED


# ============================================================
# 제어기
# ============================================================

def follower_velocity_fru(vehicle_state, now=None):
    """FC 의 자기 속도(NED)를 기체 yaw 로 돌린 (front, right, up). 출처: LOCAL_POSITION_NED(m/s) → 없거나 낡으면 GLOBAL_POSITION_INT(cm/s).
    now 를 주면 신선도(0.4 s / 0.7 s)를 본다. 둘 다 없으면 None (EKF 는 정지 가정, 시간간격 항 0)."""
    yaw = vehicle_state.get("attitude", {}).get("yaw")
    if yaw is None:
        return None
    lp = vehicle_state.get("local_position", {})
    gp = vehicle_state.get("global_position", {})
    if None not in (lp.get("vx"), lp.get("vy"), lp.get("vz")) and (now is None or is_fresh(lp, now, LOCAL_POS_MAX_AGE_SEC)):
        vx, vy, vz = float(lp["vx"]), float(lp["vy"]), float(lp["vz"])
    elif None not in (gp.get("vx"), gp.get("vy"), gp.get("vz")) and (now is None or is_fresh(gp, now, GPS_MAX_AGE_SEC)):
        vx, vy, vz = float(gp["vx"]) / 100.0, float(gp["vy"]) / 100.0, float(gp["vz"]) / 100.0
    else:
        return None
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    return np.array([vx * c + vy * s, -vx * s + vy * c, -vz])


def leader_velocity_ff(prev_ff, leader_vel_fru, dt):
    """피드포워드 항 갱신: 소프트 데드존(호버 잡음 억제) → 1차 저역통과(FF_TAU_SEC). leader_vel_fru 가 None 이면 0 으로 감쇠.

    소프트 데드존은 속도 크기에서 FF_DEADBAND_MPS 를 빼는 형태라 기울기가 1 이다. 예전의 "0.10 위로 2배까지 선형 램프" 는
    DB~2DB 구간에서 국소 기울기가 3 이라 실효 이득이 3·KFF 가 됐고, 리더 0.15 m/s 정현파에서 3.5배 증폭이 관측됐다.
    빼는 만큼 정상상태 오차가 KFF·DB/Kp 만큼 늘어(0.05 → 0.18m) 폭을 0.10 에서 0.05 로 줄였다.

    리더 속도는 IMM-EKF 가 절대 속도로 직접 추정한다(자기 속도는 예측 입력). 예전에는 자기 속도 + 상대 속도로
    만들었고 두 항의 지연 차이가 양성 되먹임이 되어 정합 저역통과(0.3s)와 긴 FF 저역통과(2.0s)가 필요했다 — 지금은
    그 경로가 없고, 긴 저역통과는 오히려 스트링 안정을 깬다(FF 가 P 응답보다 늦어 겹친다: τ 2.0s 에서 |Γ| 1.14,
    0.1s 에서 1.00). 0.1s 는 호버 잡음(v̂ σ 0.07 m/s → 명령 σ 0.03 m/s)을 눌러 주는 최소값이다. docs/STABILITY_MARGINS.md.
    """
    target = np.zeros(3)
    if leader_vel_fru is not None:
        v = np.asarray(leader_vel_fru, dtype=float)
        # 데드존은 **축별**이다. 3-노름으로 걸면 리더가 수평으로 움직이기 시작하는 순간 수직·횡 축의 잡음 억제가 사라진다(이전 감사 #37).
        # 축마다 |v_i| 에서 DB 를 빼고 부호를 되돌린다 — 한 축만 움직일 때는 노름 방식과 같다.
        target = np.sign(v) * np.maximum(np.abs(v) - FF_DEADBAND_MPS, 0.0)
    prev = np.asarray(prev_ff, dtype=float)
    a = 1.0 - math.exp(-max(float(dt), 0.0) / max(FF_TAU_SEC, 1e-3))
    return prev + a * (target - prev)


def enforce_min_separation(cmd_fru, rel_fru):
    """근접 시 두 단계로 개입한다. 1) 시선 방향 접근 속도 상한(장벽) 2) 그래도 좁혀지면 측면 회피.

    **1단계는 접근 속도의 장벽이다: 명령의 시선 방향 성분 ≤ KS·(d − d_min).** 바닥(2 m) 밖에서는 남은
    거리에 비례해 접근을 허용하고, 바닥 안에서는 침범량에 비례해 물러나게 한다 — 하나의 연속 함수다
    (제어 장벽 함수 형태). 예전에는 바닥 안에서만 적용했는데, 그 구간에서는 P 항의 후퇴 −KP·(3−d) 가
    교차점 (KS·2−KP·3)/(KS−KP) (KP 0.22 에서 1.42 m, 0.30 에서 1.00 m) 위에서 항상 더 강해 P 만 있는 경로에서는
    한 번도 물지 않았다(`test_fixes.py` `이격:` 교차점 검사).
    이 장벽이 실제로 하는 일은 **피드포워드가 파고드는 것을 막는 것**이다: 리더가 가까이 왔다가 멀어지기
    시작하면 FF 는 앞으로 밀고 P 는 뒤로 당기는데, 1.8 m 에서 리더가 0.5 m/s 로 멀어지면 P −0.36 + FF +0.40 =
    +0.04 로 바닥 안에서 접근 명령이 나온다. 장벽은 이것을 −0.12(물러남) 로 자른다. 요구도 FCR-16 은
    "바닥 안에서 접근 명령이 나가지 않는다" 이고, 그 보증은 P 가 아니라 이 장벽이 준다.

    **2단계(측면 회피)가 정면 돌진에 대한 보호다.** 리더가 MAX_VX 보다 빠르게 다가오면 정면 후퇴로는 원리적으로
    벗어날 수 없다. 느린 기체가 쓸 수 있는 유일한 회피는 비켜서는 것이라, EVADE_RADIUS_M 안에서는 시선에
    수직인 방향으로 측면 속도를 얹는다. 방향은 리더가 치우친 반대쪽 — 이미 오른쪽에 있으면 왼쪽으로 빠진다.
    수직 성분은 쓰지 않는다(MAX_VZ 0.12 로 너무 느리다). 램프는 반경 안쪽 EVADE_RAMP_M 만에 최대치에 닿는다.
    반경 전체로 훑으면 가장 위험한 근거리에서 회피가 가장 약해진다 — 첫 SITL 실측에서 0.65m 일 때 명령이
    0.125 m/s 뿐이었고 측면 속도는 0.09 m/s 였다.

    **회피의 한계(요구도 FCR-17, analysis/evasion_sim.py):** 회피는 '지나가는' 리더에 대한 것이다. 리더가 매 순간
    팔로워를 다시 겨누며 hypot(MAX_VX, MAX_VY)=0.41 m/s 보다 빠르게 추격하면 순수추격 기하상 어떤 제어기도
    접촉을 피할 수 없다. 그 위는 속도 한계(MAX_V*) 를 올리거나 리더 운용 절차로 막아야 한다.
    """
    cmd = np.asarray(cmd_fru, dtype=float)[:3].copy()
    rel = np.asarray(rel_fru, dtype=float)[:3]
    dist = float(np.linalg.norm(rel))
    if dist < 1e-6:
        return cmd

    u = rel / dist                                  # 리더 쪽 단위 벡터 (FRU)
    v_along = float(cmd @ u)                        # + 면 접근 중
    v_allowed = MIN_SEPARATION_KP * (dist - MIN_SEPARATION_M)    # 바닥 밖 +: 접근 상한, 바닥 안 −: 물러나는 속도
    if v_along > v_allowed:
        cmd = cmd + (v_allowed - v_along) * u

    global _evade_side
    if dist < EVADE_RADIUS_M:
        # 방향은 한 번만 정하고 반경을 벗어날 때까지 유지한다. 매 프레임 right 부호로 고르면, 리더가 정면일 때
        # 추정 잡음으로 부호가 뒤집혀 좌우 명령이 서로 상쇄된다 — SITL 에서 0.22 m/s 를 명령하고도 기체 측면
        # 속도는 0.01 m/s 였다. 데드밴드 안이면 부호를 보지 않고 오른쪽으로 통일한다.
        if _evade_side == 0:
            _evade_side = -1 if rel[1] > EVADE_SIDE_DEADBAND_M else 1
        frac = clamp((EVADE_RADIUS_M - dist) / EVADE_RAMP_M, 0.0, 1.0)
        cmd[1] += _evade_side * EVADE_SPEED_MPS * frac
    else:
        _evade_side = 0

    # 축별 포화 뒤에 장벽 불변식을 다시 확인한다. 포화는 성분별이라, 시선과 반대 부호인 성분이 잘리면 시선 방향 성분이 **커질** 수 있다 —
    # rel=[1.2,1.2,0]·cmd=[0.9,−0.9,0] 이면 장벽 뒤 [0.77,−1.03] 이 [0.35,−0.22] 로 잘려 v_along +0.09(접근) 가 됐다(이전 감사 #35).
    # 위반이면 초과분을 시선 방향에서 빼고 다시 포화 — 몇 번 안에 수렴한다(시선 방향 성분은 단조 감소). 상자 안에서 KS·(d−d_min) 만큼의
    # 후퇴가 불가능할 만큼 깊이 침범했을 때도 '접근하지 않는다'(v_along ≤ 0) 는 보장된다.
    lim = np.array([MAX_VX, MAX_VY, MAX_VZ])
    # 상자 안에서 시선 방향으로 낼 수 있는 최대 후퇴 속도. 허용치가 그보다 더 큰 후퇴를 요구하면(깊은 침범) 상자 한계까지만 — 그 이상을
    # 고집하면 반복이 측면 회피 성분까지 깎아 먹는다.
    nz = np.abs(u) > 1e-9
    v_box = float(np.min(lim[nz] / np.abs(u[nz]))) if np.any(nz) else 0.0
    v_target = max(v_allowed, -v_box)
    for _ in range(8):
        cmd = np.clip(cmd, -lim, lim)
        v_along = float(cmd @ u)
        if v_along <= v_target + 1e-6:
            break
        cmd = cmd + (v_target - v_along) * u
    else:
        cmd = np.clip(cmd, -lim, lim)
    if v_allowed < 0.0 and float(cmd @ u) > 0.0:
        # 마지막 보장: 바닥 안에서는 접근 성분을 0 으로 (상자 안에서 항상 가능 — 시선 성분만 빼면 크기가 줄어든다).
        cmd = np.clip(cmd - float(cmd @ u) * u, -lim, lim)
    return cmd


def coast_command_scale(range_coast_time):
    """거리 측정 코스트 중 추종 항의 배율: COAST_FADE_START_SEC 까지 1, RANGE_COAST_MAX_SEC 에서 0 으로 선형. 그 뒤는 미션이 LOST_HOLD 로
    보내 어차피 0 이다 — 이 램프가 '외삽 추정으로 2 s 전진 → 계단 정지' 를 '서서히 멈춤' 으로 바꾼다(이전 감사 #20)."""
    t = float(range_coast_time)
    if not math.isfinite(t) or t <= COAST_FADE_START_SEC:
        return 1.0
    span = max(RANGE_COAST_MAX_SEC - COAST_FADE_START_SEC, 1e-6)
    return float(clamp(1.0 - (t - COAST_FADE_START_SEC) / span, 0.0, 1.0))


def compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, pos_cov_trace, target_distance=None, leader_vel_ff=None,
                                       v_self_fru=None, coast_scale=1.0):
    """IMM 추정(FRU 상대 위치·속도) → BODY_NED [vx, vy, vz, yaw_rate].

    축마다  cmd = KFF·v_L + Kp·e + Kd·v_rel − KV·v_self.  마지막 항이 시간간격 정책이다: 전후축에서 정상상태
    (cmd = v) 는 e = (v − KFF·(v−db) + KV·v)/Kp 라 이격이 속도에 비례해 벌어진다 (0.3 m/s 에서 0.63 m). 그 대가로
    리더 속도 변동이 뒤 기체에서 증폭되지 않는다(|Γ| ≤ 1). v_self_fru 가 None(자기 속도 미수신)이면 이 항은 0 이다.
    상하축의 KV 는 KV_SELF_UP(0.15)이다. 불확실하면(pos_cov_trace) 추종 항을 감속하고, 거리 코스트 중이면 coast_scale(0~1)을 더 곱한다.
    yaw 는 둘 다 곱하지 않는다 — 측정을 되찾는 유일한 채널을 불확실할 때 느리게 만들 이유가 없다(이전 감사 #41). 이격 장벽·회피는
    배율 뒤에 적용되므로 줄지 않는다.
    """
    if target_distance is None:
        target_distance = TARGET_DISTANCE_M
    front, right, up = (float(v) for v in np.asarray(rel_fru, dtype=float)[:3])
    v_front, v_right, v_up = (float(v) for v in np.asarray(rel_vel_fru, dtype=float)[:3])
    vs_f, vs_r, vs_u = (0.0, 0.0, 0.0) if v_self_fru is None else (float(v) for v in np.asarray(v_self_fru, dtype=float)[:3])
    # 추정이 비유한이면 '모른다' 이고, 모르면 정지다. clamp 가 NaN 을 상한으로 바꾸기 전에 여기서 끊는다.
    if not all(math.isfinite(v) for v in (front, right, up, v_front, v_right, v_up, vs_f, vs_r, vs_u, float(target_distance))):
        return np.zeros(4)
    # 공분산 trace 가 NaN 이면 'NaN > 임계' 가 False 라 감속 게이트가 열린 채 지나간다 — 비유한은 최대 불확실로 본다.
    if not math.isfinite(float(pos_cov_trace)):
        pos_cov_trace = float("inf")

    if pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE:
        scale = 0.55
    elif pos_cov_trace > UNCERTAINTY_SLOWDOWN_TRACE * 0.5:
        scale = 0.75
    else:
        scale = 1.0
    scale *= float(clamp(float(coast_scale), 0.0, 1.0))

    ff_f, ff_r, ff_u = (0.0, 0.0, 0.0) if leader_vel_ff is None else (float(v) for v in np.asarray(leader_vel_ff, dtype=float)[:3])
    cmd_forward = clamp((KFF_LEADER_VEL * ff_f + KP_FORWARD * (front - float(target_distance)) + KD_FORWARD * v_front - KV_SELF * vs_f) * scale, -MAX_VX, MAX_VX)
    cmd_right = clamp((KFF_LEADER_VEL * ff_r + KP_RIGHT * right + KD_RIGHT * v_right - KV_SELF * vs_r) * scale, -MAX_VY, MAX_VY)
    cmd_up = clamp((KFF_LEADER_VEL * ff_u + KP_UP * up + KD_UP * v_up - KV_SELF_UP * vs_u) * scale, -MAX_VZ, MAX_VZ)
    # yaw: 선두 방위각을 0 으로 (시야 이탈 방지). BODY_NED yaw_rate 우회전 +, 타겟이 오른쪽이면 bearing + → 부호 일치.
    # 분모 하한을 최소 이격(2 m)으로: 그 안에서는 '횡 오차 / 2 m' 의 각도로 돌린다 — 0.5 m 앞 0.3 m 옆이면 atan2 가 31° 로 yaw 가 포화되던 것
    # (이전 감사 #56)을 8.5°·0.12 rad/s 로 눌러, 근접 회피 중 기수가 휘둘리지 않게 한다. 2 m 밖에서는 전과 같다.
    cmd_yaw_rate = clamp(KP_YAW * math.atan2(right, max(front, MIN_SEPARATION_M)), -MAX_YAW_RATE, MAX_YAW_RATE)

    cmd_fru = enforce_min_separation(np.array([cmd_forward, cmd_right, cmd_up]), rel_fru)
    return np.append(fru_to_body_ned_velocity(cmd_fru), cmd_yaw_rate)


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
    gate_ok, d2 = rel.gate_bearing2d(ekf, bearing_meas["z"], rel.R_bearing0)   # 게이트는 기본 R, 신뢰도는 이득(Rb)에만
    if gate_ok:
        ekf.update_bearing2d(bearing_meas["z"], Rb)
        return ok_label, d2
    return reject_label, d2


class InitConfirm:
    """EKF 초기화 확인: 연속 n 프레임의 RGB-D 측정이 서로 tol 안에 있어야 True. 한 프레임 오검출(배경에 앉은 깊이, 새)로 추정기를 시작하면
    그 뒤 모든 측정이 게이트에서 거부되고 2 s 코스트 뒤에야 풀렸다(이전 감사 #60). 끊기면(프레임에 후보가 없으면) 처음부터 센다."""

    def __init__(self, frames=INIT_CONFIRM_FRAMES, tol_m=INIT_CONSISTENCY_M):
        self.frames = max(int(frames), 1)
        self.tol = float(tol_m)
        self.reset()

    def reset(self):
        self.last_z = None
        self.count = 0

    def offer(self, z):
        z = np.asarray(z, dtype=float)[:3]
        if self.last_z is not None and float(np.linalg.norm(z - self.last_z)) <= self.tol:
            self.count += 1
        else:
            self.count = 1
        self.last_z = z
        if self.count >= self.frames:
            self.reset()
            return True
        return False


def fuse_vision(ekf, rel, rgbd_meas, bearing_meas, r_vis, r_depth, use_mars_imm, init_confirm=None):
    """RGB-D 3D 측정 → 게이트 통과면 위치 업데이트, 아니면 bearing 으로 강등. 반환 (update_used, gate_d2).
    init_confirm(InitConfirm)을 주면 초기화는 연속 일관 측정으로만 한다; None 이면 첫 측정으로 바로 (단위 검사·분석용)."""
    if rgbd_meas is not None and r_vis > 0.0 and r_depth > 0.0:
        range_m = float(np.asarray(rgbd_meas["z"], dtype=float)[2])        # 전방 깊이 — R 의 거리 배율
        R = rel.make_R_rgbd(r_vis, r_depth, range_m) if use_mars_imm else None
        if not ekf.initialized:
            if init_confirm is not None and not init_confirm.offer(rgbd_meas["z"]):
                return "init_pending", None
            ekf.init(rgbd_meas["z"])
            return "init_rgbd", None
        # 게이트는 **기본 R**(측정 거리에서의) 로, 신뢰도로 부풀린 R 은 갱신(칼만 이득)에만 쓴다. 같은 R 을 게이트에도 쓰면 신뢰도가 낮을수록
        # (min_reliability 0.05 → R 20배) 게이트가 함께 넓어져 품질이 나쁜 측정일수록 더 잘 통과했다(이전 감사 #26·#59, 신규 16).
        gate_ok, d2 = rel.gate_position3d(ekf, rgbd_meas["z"], rel.R_rgbd0_at(range_m) if use_mars_imm else None)
        if gate_ok or not use_mars_imm:
            ekf.update_position3d(rgbd_meas["z"], R)
            return "rgbd", d2
        # 3-D 게이트가 거부한 측정의 bearing 은 쓰지 않는다. 예전에는 같은 측정의 방위를 2-D 게이트에 넣었는데(부푼 R 로 통과),
        # 거부된 물체가 리더가 아닐 때 그 방위가 추정을 그쪽으로 끌어 결국 3-D 게이트까지 열어 버렸다(2026-10-01 감사 2번).
        # bearing 폴백은 '깊이 통계가 무효' 인 경우(아래)에만 쓴다.
        return "gate_reject_rgbd", d2
    if init_confirm is not None and not ekf.initialized:
        init_confirm.reset()          # 이번 프레임에 쓸 만한 거리 측정이 없었다 — 연속성이 끊겼다
    if USE_BEARING_FALLBACK and bearing_meas is not None and ekf.initialized:
        return _bearing_update(ekf, rel, bearing_meas, r_vis, "bearing", "gate_reject_bearing")
    return "none", None


# esp_vel_hint_used: 이름은 역사적(예전 weak hint), 값은 속도 측정이 게이트를 통과했는지. 로그 호환을 위해 키 유지.
ESP_IDLE = {"esp_update_used": "none", "esp_vel_hint_used": False, "esp_gate_d2": None,
            "esp_vel_gate_d2": None, "r_esp_gps": 0.0, "r_esp_time": 0.0}


def fuse_esp32(ekf, rel, leader_meas):
    """ESP32 GPS 상대위치를 게이트 후 위치 업데이트(source='gps')로, 상대속도는 게이트 있는 정규 칼만 속도 측정
    (apply_leader_velocity_update_to_imm → update_velocity3d)으로 반영한다. 새 패킷(rx_time 변경)일 때만 불린다 —
    같은 관측을 매 프레임 독립 측정처럼 되풀이하면 공분산이 거짓으로 줄어든다.
    반환 dict 의 esp_vel_hint_used 는 이름이 역사적이고 값은 속도 게이트 통과 여부다."""
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
        gate_ok, d2 = rel.gate_position3d(ekf, z_esp, rel.R_gps0)     # 게이트는 기본 R (위 fuse_vision 과 같은 이유)
        if gate_ok:
            ekf.update_position3d(z_esp, R_esp, source="gps")
        used = "esp_gps" if gate_ok else "gate_reject_esp_gps"

    # 상대 속도도 정규 측정으로 — 게이트를 통과한 것만 반영한다 (예전 weak hint 는 게이트를 우회했다). 패킷에 속도가 없으면 생략.
    vel_ok, vel_d2 = (False, None)
    if leader_meas.get("rel_vel_cam") is not None:
        vel_ok, vel_d2 = apply_leader_velocity_update_to_imm(ekf, leader_meas["rel_vel_cam"], rel, r_esp_time)
    return {"esp_update_used": used, "esp_vel_hint_used": vel_ok, "esp_gate_d2": d2,
            "esp_vel_gate_d2": vel_d2, "r_esp_gps": r_esp_gps, "r_esp_time": r_esp_time}


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


def reacquire_hint(ekf, intrinsics):
    """트랙이 없을 때 새 트랙을 'EKF 가 예측하는 화소 위치 근처' 에서만 잡게 하는 (uv, 반경 px). EKF 가 없거나 믿을 수 없으면 None.

    트랙을 버린 뒤(max_lost) 재획득을 '가장 큰 검출' 로 하면 0.4 s 검출 공백 한 번에 리더 신원이 화면 안의 더 큰 물체로 넘어갔다
    (2026-10-01 감사 2번). 반경은 위치 공분산을 영상에 투영한 99 % 타원 반경 + REACQ_MARGIN_PX. EKF 가 2 s 넘게 코스팅해
    is_reliable() 가 꺼지면 힌트도 없어져 예전처럼 전체 화면에서 다시 고른다.
    """
    if ekf is None or not ekf.initialized or not ekf.is_reliable():
        return None
    x, P = ekf.get_state()
    uv = camera_to_pixel(x[:3], intrinsics)
    if uv is None or not np.all(np.isfinite(uv)):
        return None
    sigma_uv = PerceptionScheduler._project_covariance_to_image(x, P, intrinsics)
    radius = math.sqrt(CHI2_2D_99 * max(float(np.max(np.linalg.eigvalsh(sigma_uv))), 1e-6)) + REACQ_MARGIN_PX
    return (float(uv[0]), float(uv[1])), float(radius)


def _opt_float(v):
    return None if v is None else float(v)


def build_log_row(s):
    """s: 루프의 프레임 상태 dict → 중첩 dict(ExperimentLogger 가 'a.b.c' 로 평탄화). 키 이름은 비행 후 로그 분석
    (저장소 밖 노트북/엑셀) 호환을 위해 바꾸지 않는다. 저장소 안에서 이 JSONL/CSV 를 읽는 스크립트는 없다.
    'time' 은 벽시계(epoch), 'mono' 는 루프가 실제로 쓰는 단조 시계다."""
    lm, esp, cmd = s["leader_meas"], s["esp"], s["current_body_cmd"]
    avail = lm.get("available", False)
    ekf = s["ekf"]
    return {
        "time": s["wall_now"], "mono": s["now"], "dt": s["dt"], "fps": s["fps_display"],
        "mars": {"enabled": s["use_mars_imm"], "policy": s["policy"]},
        "mission": {"state": s["mission_state"], "mode": s["mission_policy"]["mode"],
                    "allow_follow": s["mission_policy"]["allow_follow"], "land": s["mission_policy"]["land"]},
        "track": s["track"] or {},
        "measurement": s["rgbd_meas"] or {},
        "reliability": {"vision": s["r_vis"], "depth": s["r_depth"], "gate_d2": s["gate_d2"], "update_used": s["update_used"]},
        "leader_esp32": {
            "available": avail, "reason": lm.get("reason", "none"), "age": lm.get("age", 999.0), "seq": lm.get("seq", -1),
            "leader_alt": lm.get("leader_alt", None),
            "leader_hspeed": _opt_float(lm.get("leader_hspeed")) if avail else None,
            "leader_vz_up": _opt_float(lm.get("leader_vz_up")) if avail else None,
            "roll": lm.get("roll", None), "pitch": lm.get("pitch", None), "yaw": lm.get("yaw", None),
            "rel_fru": lm.get("rel_fru", np.zeros(3)), "rel_cam": lm.get("rel_cam", np.zeros(3)),
            "rel_vel_fru": lm.get("rel_vel_fru", np.zeros(3)), "rel_vel_cam": lm.get("rel_vel_cam", np.zeros(3)),
            **esp,
        },
        "ekf": {"x": s["x_est"], "P_trace_pos": s["pos_cov_trace"], "mu": s["mu"], "coast_time": ekf.coast_time,
                "range_coast_time": ekf.range_coast_time, "vision_range_coast_time": ekf.vision_range_coast_time,
                "initialized": ekf.initialized, "reliable": ekf.is_reliable() if ekf.initialized else False},
        "relative_fru": dict(zip(("front", "right", "up"), s["rel_fru"]), **dict(zip(("v_front", "v_right", "v_up"), s["rel_vel_fru"]))),
        "vehicle_state": {**{k: s["vehicle_state"].get(k, {}) for k in ("gps", "global_position", "local_position", "attitude",
                                                                           "system_time", "rangefinder")},
                          "gps_fresh": s["gps_fresh"], "local_position_fresh": s["local_pos_fresh"], "attitude_fresh": s["attitude_fresh"],
                          # 고도 바닥이 실제로 쓴 고도와 출처 (rangefinder | local_position | global_position | None). system_time 은 FC 로그 정렬용.
                          "follower_alt_m": s.get("follower_alt"), "alt_source": s.get("alt_source")},
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
    # 설정 모순(min_separation ≥ TARGET, 이득 음수, 모르는 lost_action …)은 여기서 ValueError — 비행 중이 아니라 시동에서 멈춘다.
    validate_config(CONFIG, target_distance_m=TARGET_DISTANCE_M, target_distance_gps_only_m=TARGET_DISTANCE_GPS_ONLY_M,
                    max_v=(MAX_VX, MAX_VY, MAX_VZ), gains=(KP_FORWARD, KD_FORWARD, KP_RIGHT, KD_RIGHT, KP_UP, KD_UP, KP_YAW))

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
    init_confirm = InitConfirm()
    mission = MissionManager(lost_action=LOST_ACTION)
    leader_rx = open_leader_receiver()
    install_signal_handlers()
    log = (ExperimentLogger(CONFIG["logger"]["log_dir"], max_mb=CONFIG["logger"].get("max_mb", 200.0),
                            fsync_sec=CONFIG["logger"].get("fsync_sec", 1.0)) if CONFIG["logger"]["enabled"] else None)

    last_track = None
    # 루프 시계는 단조 시계다. 벽시계(time.time)는 NTP 동기·수동 시각 설정으로 앞뒤로 뛰고, 앞으로 뛰면 dt·range_coast·
    # LOST_HOLD 타이머가 한꺼번에 만료돼 비행 중 LAND 가 나가고, 뒤로 뛰면 위상 고정 setpoint 스케줄러가 점프 크기만큼 송신을
    # 멈춘다(이전 감사 #48·#49). mavlink_io·leader_telemetry 의 timestamp/rx_time 도 같은 시계라 is_fresh 비교가 맞는다.
    prev_time = time.monotonic()
    last_stat_print = 0.0
    last_setpoint_time = 0.0
    last_land_send = 0.0          # C2: LAND 재시도 타이머
    prev_fc_accepts = False       # GUIDED 진입 에지 검출용
    cam_fail_streak = 0
    fc_fail_t0 = None             # FC 링크 예외가 시작된 시각 (단조). None 이면 정상
    fc_last_reconnect = 0.0
    fc_fail_last_warn = 0.0
    last_stream_request = 0.0     # request_data_streams 재요청 타이머 (연결 시 1회는 connect_fc 가 했다)
    last_ack_ts = None            # 마지막으로 보고한 COMMAND_ACK 의 timestamp
    last_statustext_ts = None
    alt_source = None
    fatal = None                  # 루프를 끝낸 치명 원인 ("camera" | "fc") — finally 에서 LOST_ACTION 을 보낼지 결정
    last_heartbeat_tx = 0.0
    fc_mode = "?"
    fc_accepts_setpoints = False
    target_distance_m = TARGET_DISTANCE_M   # 램프 상태 (TARGET_DISTANCE_RAMP_MPS)
    stall_count = 0
    fc_critical_warned = False
    last_fused_rx_time = None     # 마지막으로 EKF 에 융합한 ESP32 패킷의 rx_time
    show_window = SHOW_WINDOW
    frame_idx = 0
    fps_counter, fps_t0, fps_display = 0, time.monotonic(), 0.0
    prev_body_cmd = np.zeros(4)
    current_body_cmd = np.zeros(4)
    prev_rpy_for_comp = None      # 직전 프레임 팔로워 (roll, pitch, yaw)
    prev_att_for_comp = None      # 직전 프레임 ATTITUDE dict (yawspeed·timestamp 로 불연속 판정)
    prev_send_enabled = SEND_MAVLINK_COMMANDS
    ff_fru = np.zeros(3)          # 리더 속도 피드포워드 (FRU, 저역통과 상태)
    v_leader_fru = None           # 리더 절대 속도 추정 (FRU). STAT 진단용으로 루프 밖에서도 참조

    print("=" * 90)
    print("[INFO] q/ESC 종료 | v: MAVLink velocity 송신 on/off | l: LAND | h: HOLD")
    print(f"[INFO] SEND_MAVLINK_COMMANDS={SEND_MAVLINK_COMMANDS}  USE_LEADER_ESP32={USE_LEADER_ESP32} ({LEADER_TELEMETRY_KIND})")
    print("[INFO] 실제 비행 전 반드시 프로펠러 제거 상태에서 확인")
    print("=" * 90)

    try:
        while True:
            now = time.monotonic()
            # dt 는 직전 반복부터의 실제 경과 — 카메라·FC 실패 프레임도 루프의 나머지를 돌므로(continue 없음) 매 반복 prev_time 을 갱신한다.
            dt = max(now - prev_time, 1e-4)
            # ---------------- 루프 워치독 ----------------
            if dt > LOOP_STALL_SEC and frame_idx > 0:
                stall_count += 1
                print(f"[WARN] 루프 지연 {dt:.2f}s (누적 {stall_count}회)" + (" — FC 는 이미 정지, 명령 상태 0 에서 재시작" if dt > LOOP_RESET_SEC else ""))
                if dt > LOOP_RESET_SEC:
                    prev_body_cmd = np.zeros(4)
                    ff_fru = np.zeros(3)
                    last_setpoint_time = now - SETPOINT_PERIOD_SEC

            # ---------------- Pixhawk 수신 (예외가 이어지면 재연결, FC_FAIL_SEC 뒤 포기) ----------------
            # 실패해도 continue 하지 않는다 — 카메라·추정·미션·setpoint(송신 예외는 각자 잡는다)는 계속 돈다. 캐시된 모드는 heartbeat 신선도
            # (3 s)로 '모름' 이 되어 모드 변경 명령이 막히고, 카메라가 루프를 30 Hz 로 묶으므로 바쁜 대기도 아니다(이전 감사 #8·#52 의 FC 쪽).
            try:
                drain_messages(master)
                fc_fail_t0 = None
            except Exception as exc:
                if fc_fail_t0 is None:
                    fc_fail_t0 = now
                if now - fc_fail_last_warn >= 1.0:
                    fc_fail_last_warn = now
                    print(f"[WARN] FC link: drain 실패 {now - fc_fail_t0:.1f}s: {type(exc).__name__}: {exc}")
                if now - fc_fail_t0 >= FC_FAIL_SEC:
                    print(f"[ERR] FC 링크 {FC_FAIL_SEC:.0f}s 동안 실패 — 종료")
                    fatal = "fc"
                    break
                if now - fc_last_reconnect >= FC_RECONNECT_SEC:
                    fc_last_reconnect = now
                    new_master = reconnect_fc(master)
                    if new_master is not None:
                        master = new_master
                        last_stream_request = now
                        print("[FC] 재연결 성공")
            vehicle_state = get_vehicle_state()

            # C2: FC 모드를 매 루프 읽는다. GUIDED/OFFBOARD 를 벗어났다 = 조종사가 탈환했다 → LAND 를 보내지 않는다.
            # heartbeat 가 HEARTBEAT_MAX_AGE_SEC 넘게 안 오면 모드를 '모름' 으로 — 조용히 죽은 링크의 캐시된 GUIDED 를 믿지 않는다
            # (이전 감사 #13). setpoint 스트림은 계속 나가고(FC 가 받으면 받는 것), 모드 변경 명령만 막힌다.
            mode_fresh = is_fresh(vehicle_state.get("mode", {}), now, HEARTBEAT_MAX_AGE_SEC)
            fc_mode = vehicle_state.get("mode", {}).get("name", "?") if mode_fresh else "?"
            fc_armed = bool(vehicle_state.get("mode", {}).get("armed", False)) and mode_fresh
            fc_accepts_setpoints = fc_mode in ("GUIDED", "OFFBOARD")
            # FC 가 GUIDED/OFFBOARD 를 스스로 떠났으면(EKF failsafe·펜스·배터리·PX4 OFFBOARD 거부) 그 이유는 STATUSTEXT 로 온다 — 조종사 탈환과
            # 구별해 콘솔에 남긴다(이전 감사 #3·#10). 모드 변경·NAV_LAND 의 COMMAND_ACK 가 거부면 경고(set_mode 는 ACK 없이 True 를 돌려준다, #47).
            st_msg = vehicle_state.get("statustext", {})
            if st_msg.get("timestamp") is not None and st_msg.get("timestamp") != last_statustext_ts:
                last_statustext_ts = st_msg.get("timestamp")
                if prev_fc_accepts and not fc_accepts_setpoints:
                    print(f"[WARN] FC 가 {fc_mode} 로 바뀜 — 최근 STATUSTEXT: {st_msg.get('text')}")
            ack = vehicle_state.get("command_ack", {})
            if ack.get("timestamp") is not None and ack.get("timestamp") != last_ack_ts:
                last_ack_ts = ack.get("timestamp")
                try:
                    if int(ack.get("command", -1)) in (MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_CMD_NAV_LAND) and int(ack.get("result", 0)) != MAV_RESULT_ACCEPTED:
                        print(f"[WARN] FC 가 명령 {ack.get('command')} 을 거부 (MAV_RESULT {ack.get('result')}) — 모드 변경이 먹지 않았다")
                except (TypeError, ValueError):
                    pass

            # GUIDED 진입 = 조종사가 방금 자동에게 넘긴 순간. 그 전까지 FC 는 우리 명령을 버렸으므로 그동안 쌓인
            # 상태(수동 상승 중 FAILSAFE_LAND 로 래치된 미션, 포화된 평활 버퍼)를 들고 들어가면 안 된다.
            # 'v' 키로 송신을 켠 순간도 같다 — dry-run 동안 미션은 계속 돌았으므로(리더를 봤다 10 s 놓치면 FAILSAFE_LAND 로
            # 래치) 그 상태로 송신을 켜면 첫 프레임에 LAND 가 나간다(2026-10-01 감사 3번).
            send_turned_on = SEND_MAVLINK_COMMANDS and not prev_send_enabled
            prev_send_enabled = SEND_MAVLINK_COMMANDS
            if (fc_accepts_setpoints and not prev_fc_accepts) or send_turned_on:
                mission.reset()
                prev_body_cmd = np.zeros(4)
                ff_fru = np.zeros(3)
                reset_evade_side()
                last_land_send = 0.0
                target_distance_m = TARGET_DISTANCE_M
                print(f"[SYS] {'송신 ON' if send_turned_on else fc_mode + ' 진입'} — 미션/명령 리셋")
            prev_fc_accepts = fc_accepts_setpoints

            gps_fresh = is_fresh(vehicle_state.get("gps", {}), now, GPS_MAX_AGE_SEC)
            gpos_fresh = is_fresh(vehicle_state.get("global_position", {}), now, GPS_MAX_AGE_SEC)
            # FC 가 failsafe 중(MAV_STATE CRITICAL/EMERGENCY)인데 모드는 GUIDED 그대로일 수 있다(배터리 경고 action 0, FS_OPTIONS 비트 2).
            # 컴패니언은 모드 변경을 하지 않지만 STAT/HUD 에 경보한다 — 조종사가 개입할 신호.
            fc_sys_status = vehicle_state.get("mode", {}).get("system_status")
            fc_critical = mode_fresh and fc_sys_status is not None and int(fc_sys_status) >= MAV_STATE_CRITICAL
            if fc_critical and not fc_critical_warned:
                print(f"[WARN] FC system_status={fc_sys_status} (CRITICAL 이상) — FC 가 failsafe 중. 조종사 개입 필요")
            fc_critical_warned = fc_critical
            local_pos_fresh = is_fresh(vehicle_state.get("local_position", {}), now, LOCAL_POS_MAX_AGE_SEC)
            attitude_fresh = is_fresh(vehicle_state.get("attitude", {}), now, ATTITUDE_MAX_AGE_SEC)
            # heartbeat 는 오는데 위치·자세 스트림이 없으면 스트림 요청이 유실된 것(FC 재부팅·라우터 재연결) — 5 s 마다 다시 요청.
            if mode_fresh and not (local_pos_fresh or gpos_fresh) and now - last_stream_request >= STREAM_REREQUEST_SEC:
                last_stream_request = now
                try:
                    request_data_streams(master)
                    print("[WARN] 위치 스트림 없음 — 데이터 스트림 재요청")
                except Exception as exc:
                    print(f"[WARN] FC link: 스트림 재요청 실패: {type(exc).__name__}: {exc}")

            # ---------------- ESP32 수신 (표시·미션용은 매 프레임, 융합은 새 패킷만) ----------------
            leader_packet = leader_rx.read_latest() if leader_rx is not None else None
            leader_meas = build_leader_measurement_from_packet(
                packet=leader_packet, follower_vehicle_state=vehicle_state, now=now,
                max_age_sec=LEADER_MAX_AGE_SEC, leader_velocity_frame=LEADER_VELOCITY_FRAME)

            if now - last_heartbeat_tx >= 1.0:
                last_heartbeat_tx = now
                try:
                    send_heartbeat(master)
                except Exception as exc:
                    print(f"[WARN] FC link: heartbeat 송신 실패: {type(exc).__name__}: {exc}")

            if now - last_stat_print >= 1.0:
                p_cv, p_ct = ekf.get_model_probs() if ekf.initialized else (0.0, 0.0)
                print(f"[STAT] {battery_text()} FPS={fps_display:.1f} MARS={'ON' if use_mars_imm else 'OFF'} "
                      f"CMD={'ON' if SEND_MAVLINK_COMMANDS else 'DRY'} FC={fc_mode}{'*' if fc_armed else ''} "
                      f"ESP={int(leader_meas.get('available', False))}:{leader_meas.get('reason', 'none')} "
                      f"CV={p_cv:.2f} CT={p_ct:.2f} coast={ekf.coast_time:.1f}s rcoast={ekf.range_coast_time:.1f}s "
                      f"mission={mission.state} "
                      f"vL={(float(np.linalg.norm(v_leader_fru)) if v_leader_fru is not None else float('nan')):.2f} "
                      f"ff={ff_fru[0]:+.2f} fresh=LP{int(local_pos_fresh)}/ATT{int(attitude_fresh)} alt={alt_source or '-'} "
                      f"{stream_rates_text(now)}" + (f" drain_overflow={mavlink_io.drain_overflows}" if mavlink_io.drain_overflows else ""))
                if SEND_MAVLINK_COMMANDS and not fc_accepts_setpoints:
                    print(f"[WARN] FC mode={fc_mode}: setpoint 는 계속 보내지만 FC 가 버린다 (ArduPilot: GUIDED / PX4: OFFBOARD 필요)")
                last_stat_print = now

            # ---------------- 카메라 (예외·None 은 드롭, 연속 실패면 포기) ----------------
            try:
                color_image, depth_image = cam.get_frames()
            except Exception as exc:
                color_image, depth_image = None, None
                print(f"[WARN] 카메라 프레임 실패 {cam_fail_streak + 1}회: {type(exc).__name__}: {exc}")
            frame_ok = color_image is not None   # 예외도, 컬러/깊이 결손(None)도 같은 드롭 — 둘 다 연속 실패로 센다
            if not frame_ok:
                cam_fail_streak += 1
                if cam_fail_streak >= CAM_FAIL_LIMIT:
                    print(f"[ERR] 카메라 연속 실패 {CAM_FAIL_LIMIT}회 — 종료 (LOST_ACTION={LOST_ACTION})")
                    fatal = "camera"
                    break
                # 실패 프레임도 루프의 나머지(예측·소실 타이머·미션·setpoint)는 돈다 — 예전의 continue 는 setpoint 송신과 소실 타이머를
                # 통째로 건너뛰어 FC 가 마지막 속도를 3 s 유지하고 소실 판정이 얼어붙었다(이전 감사 #8·#52). 카메라가 죽으면 '리더를 못
                # 본다' 와 같고, 그 경로(코스트 → LOST_HOLD → 10 s 뒤 LOST_ACTION)가 그대로 적용된다.
            else:
                cam_fail_streak = 0
                frame_idx += 1
                fps_counter += 1

            prev_time = now
            if now - fps_t0 >= 1.0:
                fps_display = fps_counter / max(now - fps_t0, 1e-6)
                fps_counter, fps_t0 = 0, now

            # ---------------- IMM predict (팔로워 자세 변화만큼 상대상태를 역회전한 뒤) ----------------
            att = vehicle_state.get("attitude", {})
            roll_lvl = pitch_lvl = 0.0            # 제어 입력 레벨링용. 자세가 신선하지 않으면 수평으로 본다
            if attitude_fresh and att.get("yaw") is not None:
                cur_rpy = (float(att.get("roll") or 0.0), float(att.get("pitch") or 0.0), float(att["yaw"]))
                roll_lvl, pitch_lvl = cur_rpy[0], cur_rpy[1]
                if prev_rpy_for_comp is not None and ekf.initialized:
                    if attitude_jump(prev_att_for_comp, att):
                        # FC 의 yaw 재정렬: 기체(카메라)는 돌지 않았으므로 상대 상태는 그대로가 맞다. 보상을 건너뛴다.
                        print(f"[WARN] ATTITUDE yaw 불연속 {math.degrees(cur_rpy[2] - prev_rpy_for_comp[2]):+.1f}° — 자세 보상 생략")
                    else:
                        ekf.compensate_ego_rotation(ego_rotation_cam(prev_rpy_for_comp, cur_rpy))
                prev_rpy_for_comp = cur_rpy
                prev_att_for_comp = att
            else:
                prev_rpy_for_comp = None
                prev_att_for_comp = None
            # 자기 속도는 EKF 예측의 입력이다 (상대 위치 = ∫(리더 절대 속도 − 자기 속도)). 보상(위에서 ego_vel 도
            # 함께 회전) 뒤, predict 앞에 넣어야 프레임이 맞는다. 초기화 전에도 넣는다 — init 이 초기 절대 속도로 쓴다.
            # follower_velocity_fru 는 yaw 만 돌린 수평 프레임이다. EKF 의 카메라 프레임은 roll/pitch 까지 돌아 있으므로 수평 → 기체 → 카메라
            # 로 넣는다(이전 감사 #38·#44: 기울어진 채로 두 프레임이 어긋났다).
            v_self_fru = follower_velocity_fru(vehicle_state, now) if attitude_fresh else None     # LOCAL_POSITION_NED → GLOBAL_POSITION_INT 폴백
            ekf.set_ego_velocity_cam(level_fru_to_camera_xyz(v_self_fru, roll_lvl, pitch_lvl) if v_self_fru is not None else None)
            if ekf.initialized:
                ekf.predict(dt)

            # ---------------- 스케줄러 → 검출/추적 (프레임이 없으면 '검출 없음' 프레임으로) ----------------
            if not frame_ok:
                policy = {"run_detector": False, "use_full_frame": True, "roi": None, "detect_every": 1, "reason": "camera_fail"}
                track = None
                rgbd_meas = bearing_meas = None
                r_vis = r_depth = 0.0
                update_used, gate_d2 = "camera_fail", None
            else:
                if use_mars_imm:
                    policy = scheduler.decide(ekf.get_state_dict(), color_image.shape, intrinsics, last_track)
                else:
                    policy = {"run_detector": True, "use_full_frame": True, "roi": None, "detect_every": 1, "reason": "baseline_full_frame"}

                if policy["run_detector"]:
                    # 트랙이 없을 때의 재획득은 EKF 예측점 근처에서만 (reacquire_hint). 트랙이 있으면 힌트는 쓰이지 않는다.
                    hint = reacquire_hint(ekf, intrinsics) if tracker.track is None else None
                    track = tracker.update(detector.detect(color_image, roi=None if policy["use_full_frame"] else policy["roi"]),
                                           reacquire_hint=hint)
                else:
                    track = tracker.predict_only()
                last_track = track
                if set_ae_roi is not None:
                    # 짧은 미스에는 마지막 bbox 를 유지하고, 연속 AE_ROI_RELEASE_LOST 프레임 넘게 놓쳤을 때만 전체 프레임으로.
                    keep_bbox = track is not None and int(track.get("lost_count", 0)) < AE_ROI_RELEASE_LOST
                    set_ae_roi(track["bbox"] if keep_bbox else None, now)

                # ---------------- 측정 → 융합 ----------------
                rgbd_meas = meas_builder.build_rgbd(track, depth_image)
                bearing_meas = meas_builder.build_bearing(track) if track is not None else None
                r_vis = rel.vision_reliability(rgbd_meas or bearing_meas or track)
                r_depth = rel.depth_reliability(rgbd_meas)
                update_used, gate_d2 = fuse_vision(ekf, rel, rgbd_meas, bearing_meas, r_vis, r_depth, use_mars_imm,
                                                   init_confirm=init_confirm if use_mars_imm else None)
            # 3-D 게이트 결과를 트래커에 되먹인다: 거리 측정이 연속 GATE_REJECT_DROP_FRAMES 회 거부되면 트랙을 버려
            # (리더가 아닐 가능성) 다음 프레임에 EKF 예측점 근처에서 다시 잡게 한다. 수용되면 거부 횟수를 0 으로.
            if update_used == "rgbd":
                tracker.note_gate_accept()
            elif update_used == "gate_reject_rgbd" and ekf.is_reliable():
                tracker.note_gate_reject(GATE_REJECT_DROP_FRAMES)
            # 게이트가 거부한 프레임은 '측정 없음' 과 같다 — coast_time 을 센다. 예전엔 트랙이 살아 있으면 거부가 이어져도 is_reliable() 이 영원히
            # 참이라 재획득 힌트가 틀린 예측 주위에 갇혔다(이전 감사 #55). 2 s 뒤 is_reliable 이 꺼지면 전체 화면에서 다시 고른다.
            if update_used.startswith("gate_reject") and ekf.initialized:
                ekf.on_lost(dt)

            # read_latest() 는 새 패킷이 없으면 같은 패킷을 다시 돌려준다. 같은 관측을 매 프레임 독립 측정처럼
            # 융합하면 안 되므로 새 패킷(rx_time 변경)일 때만 융합한다.
            esp = ESP_IDLE
            esp_rx_time = leader_meas.get("rx_time", None)
            # 팔로워 GLOBAL_POSITION_INT·ATTITUDE 가 신선할 때만 융합한다 — 상대위치는 그 둘로 만들어지므로 낡으면 '고정된 가짜 목표' 가 된다
            # (2026-10-01 감사 10번).
            if USE_LEADER_ESP32 and leader_meas.get("available", False) and esp_rx_time is not None \
                    and esp_rx_time != last_fused_rx_time and gpos_fresh and attitude_fresh:
                last_fused_rx_time = esp_rx_time
                esp = fuse_esp32(ekf, rel, leader_meas)

            if (track is None or track.get("is_lost", False)) and ekf.initialized:
                ekf.on_lost(dt)

            # ---------------- 추정 상태 → 미션 ----------------
            x_est, P_est = ekf.get_state()
            pos_cov_trace = float(np.trace(P_est[:3, :3])) if ekf.initialized else 999.0
            if not math.isfinite(pos_cov_trace):        # NaN 공분산 = 최대 불확실 (미션 게이트 > 8.0 이 LOST_HOLD 로 보낸다)
                pos_cov_trace = 999.0
            mu = ekf.get_model_probs() if ekf.initialized else np.array([0.0, 0.0])
            # 제어·미션 입력은 수평(기수 정렬) 프레임 — 카메라 마운트와 기체 roll/pitch 를 편다 (camera_to_level_fru). 위치에는 레버암을 더한다.
            rel_fru = camera_to_level_fru(x_est[:3], roll_lvl, pitch_lvl, position=True)
            # 상태의 속도는 리더 '절대' 속도. 제어 D 항·미션 폴백은 상대 속도(절대 − 자기)를 쓴다.
            rel_vel_fru = camera_to_level_fru(ekf.relative_velocity(), roll_lvl, pitch_lvl) if ekf.initialized else np.zeros(3)

            follower_alt, alt_source = get_follower_altitude_m(vehicle_state, now)     # 거리계 → LOCAL_POSITION_NED → GLOBAL_POSITION_INT
            # 착륙 판정용 선두 고도는 AGL 근사여야 한다 (ESP32 alt 는 절대고도라 landing_z_thresh 와 비교 불가)
            # → 팔로워 AGL(LOCAL_POSITION_NED) + 상대고도.
            leader_alt_est = float(follower_alt + rel_fru[2]) if (follower_alt is not None and ekf.initialized and ekf.is_reliable()) else None
            esp_visible = bool(leader_meas.get("available", False))
            # 패킷에 속도가 없으면(leader_vel_enu None) EKF 절대 속도로 떨어진다 — 0 을 넣으면 미션이 '리더 정지' 로 오판한다.
            leader_vel_world = (np.asarray(leader_meas["leader_vel_enu"], dtype=float)
                                if esp_visible and leader_meas.get("leader_vel_enu") is not None else None)

            # 미션의 "리더가 보인다" = 거리를 아는가. bbox 유무나 is_reliable()(bearing-only 로도 참)은 거리 관측을
            # 보장하지 않아 깊이가 죽어도 소실 판정이 안 나기 때문이다. RGB-D 와 ESP32 위치만 range_coast 를 되돌린다.
            # 추정 거리가 MAX_FOLLOW_DIST_M 보다 멀면 거리를 안다고 치지 않는다 — 먼 오검출로 초기화된 추정을 전속으로 쫓지 않게.
            range_m = float(np.linalg.norm(rel_fru)) if ekf.initialized else float("inf")
            range_too_far = ekf.initialized and range_m > MAX_FOLLOW_DIST_M
            leader_visible_for_mission = bool(ekf.has_range_fix()) and not range_too_far
            # 추종 거리는 거리의 출처로: 카메라 깊이가 살아 있으면 3m, ESP32 GPS 뿐이면 오차 여유를 둔 8m. 전환은 램프(계단이면 P 항이 뛴다).
            vision_range_ok = bool(ekf.has_vision_range_fix())
            target_goal = TARGET_DISTANCE_M if vision_range_ok else TARGET_DISTANCE_GPS_ONLY_M

            # 리더 절대 속도(FRU) 는 EKF 상태 그대로다 (자기 속도가 예측 입력으로 들어가 있다). 미션(출발/정지/착륙
            # 판단)과 피드포워드가 같이 쓴다. 상대 속도만 보면 후미가 선두 속도를 맞추는 순간 0 이 되어 '선두 정지' 로
            # 오판한다. 자기 속도·자세가 신선하고 EKF 가 신뢰할 수 있을 때만 쓰고(아니면 상태 v 는 상대 속도에 불과),
            # 없으면 미션은 상대 속도로 폴백한다.
            v_leader_fru = None
            if ekf.initialized and ekf.is_reliable() and attitude_fresh and v_self_fru is not None:
                v_leader_fru = camera_to_level_fru(ekf.leader_velocity(), roll_lvl, pitch_lvl)

            mission_state, mission_policy = mission.update(
                now=now, leader_visible=leader_visible_for_mission,
                rel_vel_est=rel_vel_fru if ekf.initialized else None,
                leader_alt=leader_alt_est, leader_vel_world=leader_vel_world, pos_cov_trace=pos_cov_trace,
                leader_vel_body=v_leader_fru)

            # ---------------- 명령 ----------------
            # 목표 이격 램프: 추종 중(allow_follow)에만 움직인다. 소실 홀드 중에 8 m 로 올라가 버리면 재획득 순간 후퇴 과도가 생긴다.
            if mission_policy["allow_follow"]:
                target_distance_m += clamp(target_goal - target_distance_m, -TARGET_DISTANCE_RAMP_MPS * dt, TARGET_DISTANCE_RAMP_MPS * dt)
            else:
                reset_evade_side()      # 추종이 끊긴 동안 회피 방향 래치를 들고 있지 않는다 — 재개 시 리더 쪽으로 비켜설 수 있었다(이전 감사 #59)
            # 리더 속도 피드포워드: 거리를 아는 추종 상태에서만. 아니면 0 으로 감쇠.
            ff_fru = leader_velocity_ff(
                ff_fru, v_leader_fru if (mission_policy["allow_follow"] and ekf.has_range_fix()) else None, dt)

            desired_body_cmd = np.zeros(4)
            failsafe = mission_policy.get("failsafe")          # None | "LAND" | "RTL"
            # 모드 변경은 FC 가 setpoint 를 받는 동안에만 스트림을 대신한다. 수동 모드에서는 스트림을 계속 흘려 PX4 의 OFFBOARD
            # 진입 전제를 깨지 않는다(2026-10-01 감사 14번: 수동 중 FAILSAFE 래치가 스트림을 끊어 OFFBOARD 전환이 거부됐다).
            suppress_stream = failsafe is not None and fc_accepts_setpoints
            if failsafe is not None:
                # C2: 모드 게이트가 곧 latch 다 — LAND/RTL 이 먹으면 FC 가 GUIDED 를 벗어나 이 분기가 더 실행되지
                # 않는다. 여전히 여기 있다는 건 명령이 먹지 않았다는 뜻이라 그때만 LAND_RETRY_SEC 간격으로 재시도
                # (조종사 탈환 시엔 fc_accepts_setpoints=False 라 아예 보내지 않는다).
                if SEND_MAVLINK_COMMANDS and fc_accepts_setpoints and now - last_land_send >= LAND_RETRY_SEC:
                    try:
                        send_failsafe(master, failsafe)
                    except Exception as exc:
                        print(f"[WARN] FC link: {failsafe} 송신 실패: {type(exc).__name__}: {exc}")
                    last_land_send = now
                    last_setpoint_time = now
            elif mission_policy["allow_follow"] and ekf.initialized and not range_too_far:
                # 거리 코스트 중이면 추종 항을 서서히 0 으로 (coast_command_scale) — 장벽·회피·yaw 는 그대로.
                desired_body_cmd = compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, pos_cov_trace, target_distance_m, ff_fru,
                                                                      v_self_fru=v_self_fru,
                                                                      coast_scale=coast_command_scale(ekf.range_coast_time))

            # 수직 축은 리더 상대 고도를 따라간다 — 트래커가 지면의 무언가를 물면 계속 하강한다. 고도 바닥(모르면 하강 금지).
            desired_body_cmd = enforce_agl_floor(desired_body_cmd, follower_alt)

            current_body_cmd = smooth_velocity_cmd(prev_body_cmd, desired_body_cmd, alpha=0.28, dt=dt)
            prev_body_cmd = current_body_cmd.copy()

            # 속도 setpoint 에는 모드 게이트를 걸지 않는다: GUIDED/OFFBOARD 가 아니면 FC 가 조용히 버리고, PX4 는
            # OFFBOARD 진입 전에 이 스트림이 먼저 흐르고 있어야 한다. 조종사를 뺏는 건 모드 변경(LAND)이며 그쪽만 막는다.
            if not suppress_stream:
                if failsafe is None:
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
            if show_window and frame_ok and frame_idx % DISPLAY_EVERY == 0:
                H, W = color_image.shape[:2]
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
                    (f"fresh GPS/LP/ATT={int(gps_fresh)}/{int(local_pos_fresh)}/{int(attitude_fresh)} alt={alt_source or '-'}"
                     + (f"  !! FC system_status={fc_sys_status}" if fc_critical else "") + (f"  stall={stall_count}" if stall_count else ""),
                     (0, 0, 255) if fc_critical else (180, 180, 255), 0.48),
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
                elif key == ord("v"):
                    # ON 은 루프 머리의 리셋(send_turned_on)을 거친다. OFF 는 마지막 setpoint 를 FC 가 GUID_TIMEOUT 3 s 동안
                    # 유지하지 않도록 HOLD(0) 를 한 번 보내고 끈다.
                    if SEND_MAVLINK_COMMANDS:
                        try:
                            send_hold(master)
                        except Exception as exc:
                            print(f"[WARN] FC link: HOLD 송신 실패: {type(exc).__name__}: {exc}")
                    SEND_MAVLINK_COMMANDS = not SEND_MAVLINK_COMMANDS
                    print(f"[SYS] SEND_MAVLINK_COMMANDS -> {SEND_MAVLINK_COMMANDS}")
                elif key == ord("h"):
                    if SEND_MAVLINK_COMMANDS:
                        send_hold(master)
                        print("[SYS] manual HOLD 송신 (다음 setpoint에 덮임)")
                    else:
                        print("[SYS] manual HOLD 생략 — CMD=DRY")
                elif key == ord("l"):
                    # 미션 경로와 같은 조종사 우선 게이트: FC 가 GUIDED/OFFBOARD 가 아니면(조종사가 탈환) 키로도 LAND 를 안 보낸다.
                    if not SEND_MAVLINK_COMMANDS:
                        print("[SYS] manual LAND 생략 — CMD=DRY ('v'로 켜야 나감)")
                    elif not fc_accepts_setpoints:
                        print(f"[SYS] manual LAND 생략 — FC mode={fc_mode} (조종사 우선)")
                    else:
                        send_land(master)
                        print("[SYS] manual LAND 송신")

            # ---------------- 로그 ----------------
            if log is not None:
                log.log(build_log_row(dict(ff_fru=ff_fru, wall_now=time.time(),
                    now=now, dt=dt, fps_display=fps_display, use_mars_imm=use_mars_imm, policy=policy,
                    mission_state=mission_state, mission_policy=mission_policy, track=track, rgbd_meas=rgbd_meas,
                    r_vis=r_vis, r_depth=r_depth, gate_d2=gate_d2, update_used=update_used, leader_meas=leader_meas,
                    esp=esp, x_est=x_est, pos_cov_trace=pos_cov_trace, mu=mu, ekf=ekf, rel_fru=rel_fru,
                    rel_vel_fru=rel_vel_fru, vehicle_state=vehicle_state, gps_fresh=gps_fresh,
                    local_pos_fresh=local_pos_fresh, attitude_fresh=attitude_fresh, current_body_cmd=current_body_cmd,
                    target_distance_m=target_distance_m, vision_range_ok=vision_range_ok, follower_alt=follower_alt, alt_source=alt_source)))

    except KeyboardInterrupt:
        print("\n[SYS] KeyboardInterrupt")

    finally:
        print(f"[SYS] shutdown{' (fatal=' + fatal + ')' if fatal else ''}")
        try:
            if SEND_MAVLINK_COMMANDS and master is not None:
                if fatal is not None and fc_accepts_setpoints:
                    # 컴패니언이 죽으면 ArduCopter 는 GUID_TIMEOUT 뒤 그 자리에서 무한 호버한다(이전 감사 #1·#50·#53). 카메라 사망은
                    # '리더를 영영 못 본다' 이므로 소실과 같은 행동(LOST_ACTION)을 한 번 보내고 나간다. FC 링크 사망도 같은 시도를 한다 —
                    # 닿을 가능성은 낮지만 비용이 없다. 조종사가 탈환한 상태(마지막으로 안 모드 기준)면 보내지 않는다.
                    act = {"land": "LAND", "rtl": "RTL", "hold": None}.get(LOST_ACTION)
                    if act is not None:
                        print(f"[SYS] {fatal} 사망 종료 — {act} 송신")
                        send_failsafe(master, act)
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
