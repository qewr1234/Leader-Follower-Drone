"""
config.py — MARS-IMM 설정값

MARS-IMM:
Mode-Aware Reliability-Scheduled IMM Tracking
"""

import os

# 모델 파일 위치. 기체마다 다르면 환경변수로: MARS_MODEL_DIR=/path python3 main.py
MODEL_DIR = os.environ.get("MARS_MODEL_DIR", "/home/dsl/DRONE")

CONFIG = {
    "detector": {
        "pt_model": os.path.join(MODEL_DIR, "leader_drone_yolo11n.pt"),
        # 같은 Jetson 에서 `yolo export model=<pt> format=engine imgsz=416 half=True` 로 생성.
        # 이 파일이 있으면 TensorRT, 없으면 .pt 로 폴백한다 (시작 로그 [YOLO] backend= 로 확인).
        "trt_model": os.path.join(MODEL_DIR, "leader_drone_yolo11n.engine"),
        # 현재는 사람으로 실험 중. 드론 전환 시 "leader_drone"으로 변경.
        # (커스텀 모델(.pt/.engine)에 해당 클래스가 있어야 함)
        "target_class_name": "person",
        "conf_thres": 0.25,
        "iou_thres": 0.45,
        "imgsz": 416,
    },
    "camera": {
        "width": 640,
        "height": 480,
        "fps": 30,
        "depth_min_m": 0.30,
        "depth_max_m": 10.00,   # C4: TARGET_DISTANCE_M(3.0) 대비 여유 7m
        # ---- 실외 노출 (컬러 센서 librealsense 옵션. 펌웨어/버전이 미지원인 옵션은 건너뛰고 로그) ----
        # False: 저조도에서도 30fps 유지(노출 상한 = 1/fps). True 면 AE 가 fps 를 떨어뜨려 제어 주기가 흔들린다.
        "color_auto_exposure_priority": False,
        # AE 노출 상한(µs). 이동 중 모션 블러 억제. 0 이면 상한 없음. (auto_exposure_limit, librealsense ≥ 2.50)
        "color_exposure_max_us": 8000,
        # 하늘 배경 역광에서 어두운 피사체 쪽으로 노출 보정.
        "color_backlight_compensation": True,
        # AE 측광 영역을 추적 bbox(1.5배)로 옮긴다(≤1Hz). 하늘 평균이 아니라 리더에 노출을 맞춘다. 소실 시 전체로 복귀.
        "ae_roi_follow_track": True,
    },
    "measurement": {
        "bbox_inner_ratio": 0.55,
        "min_depth_valid_count": 20,
        "min_depth_valid_ratio": 0.15,
        "max_depth_mad": 0.50,
    },
    "imm": {
        "max_coast_sec": 2.0,
        # 거리 측정이 끊긴 뒤 소실로 보기까지의 유예.
        # 미션 총 착륙 지연 = 이 값 + MissionManager.lost_hold_sec
        "range_coast_max_sec": 2.0,
        "sigma_xy": 0.15,
        "sigma_z": 0.25,
    },
    "reliability": {
        "min_reliability": 0.05,
        "mahalanobis_threshold_3d": 11.34,  # chi-square df=3, p≈0.99
        "mahalanobis_threshold_2d": 9.21,   # chi-square df=2, p≈0.99
        "base_R_rgbd_diag": [0.15**2, 0.15**2, 0.25**2],
        "base_R_bearing_diag": [0.03**2, 0.03**2],
        "base_R_gps_diag": [2.0**2, 2.0**2, 3.0**2],
    },
    "scheduler": {
        "enable_roi": True,
        "min_roi_size": 192,
        "base_roi_size": 320,
        "max_roi_size": 640,
        "lost_full_frame_threshold": 5,
        "full_frame_interval": 20,
        "normal_detect_every": 2,
        "maneuver_detect_every": 1,
    },
    "controller": {
        "uncertainty_slowdown_trace": 4.0,
        # 리더 속도 피드포워드: cmd = KFF·v_leader + Kp·e + Kd·v_rel.
        # 없으면 정상상태 거리 오차 = v_leader/Kp (0.3m/s→1.4m, 1m/s→4.5m 로 깊이창 10m 밖으로 밀려 소실).
        # KFF<1 로 두는 이유는 EKF 속도 지연을 통한 자기 속도 양성 되먹임의 이득 여유 (main.leader_velocity_ff 주석).
        "leader_vel_ff_gain": 0.8,
        # 피드포워드 1차 저역통과. 0.7s 였을 때 이득여유 4.9dB·1.15rad/s 에서 리더 속도 변동 1.8배 증폭(스트링 불안정) —
        # 자기 속도 되먹임 경로가 P+D 루프의 위상 교차와 겹쳐서다. 2.0s 로 그 경로를 공진 아래로 내린다
        # (docs/STABILITY_MARGINS.md 7절: GM 14.4dB, |Γ| 1.13). 정상상태 오차는 변하지 않는다.
        "leader_vel_ff_tau_sec": 2.0,
        # 자기 속도(FC) 정합 저역통과. EKF 상대속도 추정은 63% 응답에 0.30s(실측) 걸리는데 FC 자기 속도는 바로 들어오므로,
        # 같은 지연으로 늦춰야 v_leader = v_self + v_rel 에서 자기 속도가 상쇄된다. 0 이면 필터 없음.
        "leader_vel_ff_self_tau_sec": 0.3,
        # 소프트 데드존: |v| ≤ 이 값이면 0, 그 위는 크기에서 이 값을 뺀다(기울기 1). 예전 0.10 램프(2배까지 선형)는 국소 기울기가
        # 3 이라 리더 0.1~0.2 m/s 에서 실효 KFF 가 2.4 가 됐다. 빼는 만큼 정상상태 오차가 KFF·DB/Kp 늘므로(0.10 → +0.36m)
        # 폭을 0.05 로 줄였다. 호버 잡음은 위의 2.0s 저역통과가 평균내므로 데드존은 바이어스만 막으면 된다.
        "leader_vel_ff_deadband_mps": 0.05,
        # 제어 오차 수평화. EKF 상대위치는 기체 고정 카메라 프레임(roll/pitch 포함)인데 FC 는 BODY_NED 속도를 yaw 만으로
        # 회전한다(ArduCopter body_to_earth2D, PX4 mavlink_receiver — z 는 그대로). 그래서 pitch 10° 로 기운 채 같은 고도
        # 리더를 보면 vz 0.094 m/s(상한 0.12) 가 나간다. True 면 제어 직전에 roll/pitch 를 되돌린다(main.level_fru_by_roll_pitch).
        # 기본 True (2026-09-24). 폐루프 하네스 tilt 시나리오(test_closed_loop.py --scenario tilt --level 0/1)가 꺼진 상태의
        # 결함(pitch −10° 에 vz −0.092, 팔로워가 D·tan10° = 0.52 m 위로 올라가 정착)과 켠 상태의 해소(|vz| < 0.01)를 재현한다.
        # 맞바람에 기운 채 호버하면 이 편향이 상시 걸린다. 실기 전 지상 기울임 점검은 docs/FLIGHT_SAFETY_CHECKLIST.md.
        "level_by_attitude": True,
    },
    # ---- 편대 (선두 1 : 후미 N 토대, formation.py) ----
    # 기본값은 슬롯 미설정 = 후미 자신의 시선 기준 리더 뒤 TARGET_DISTANCE_M (기존 동작과 동일).
    "formation": {
        "follower_id": os.environ.get("MARS_FOLLOWER_ID", "F1"),
        # 기대하는 리더 ID (ArduPilot FOLL_SYSID 역할). "" 이면 검사하지 않는다. 패킷의 leader_id/id/sysid 와 비교.
        "leader_id": os.environ.get("MARS_LEADER_ID", ""),
        # follower_id → 슬롯. offset 은 리더 → 슬롯 벡터. frame: "leader" = 리더 heading 기준 [front, right, up]
        # (ArduPilot FOLL_OFS_TYPE=1) / "ned" = [north, east, down] (FOLL_OFS_TYPE=0) / "los" = 후미 시선 기준(기존).
        # 예)  "F1": {"offset": [-3.0, 0.0, 0.0], "frame": "leader", "slot_id": "tail"},
        #      "F2": {"offset": [-3.0, 2.5, 0.0], "frame": "leader", "slot_id": "right_wing"},
        #      "F3": {"offset": [-3.0, -2.5, 0.0], "frame": "leader", "slot_id": "left_wing"},
        "slots": {},
        # 슬롯 정적 검사(formation.validate_formation): 슬롯 간 최소 이격, 깊이창 여유(C4 의 목표+v/Kp 여유).
        "min_separation_m": 2.0,
        "depth_reserve_m": 3.0,
        # 리더 heading 을 속도 방향에서 얻을 때의 최소 수평 속도(PX4 follow_me 는 1.0 m/s). 그 아래는 최근값을 hold_sec 유지.
        "heading_min_speed_mps": 0.5,
        "heading_hold_sec": 2.0,
        # 피드포워드 소스. "vision" = 자기 속도 + EKF 상대 속도(기존, 자기 속도 양성 되먹임 경로 있음 → KFF<1 필요),
        # "broadcast" = 선두가 방송한 절대 속도만(없으면 P+D 만), "auto" = 방송이 있으면 방송, 없으면 vision.
        # 체인·다중 후미에서는 broadcast/auto 가 맞다 (docs/MULTI_FOLLOWER_FOUNDATION.md, Seiler 2004 / Zheng 2016).
        "ff_source": "vision",
    },
    "logger": {
        "enabled": True,
        "log_dir": "logs",
    },
}
