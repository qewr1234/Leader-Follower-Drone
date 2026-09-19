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
        # CT 모델의 회전율을 추정할 최소 리더 속도. 상대가 아니라 절대 속도 기준이라 추종 중에도 유효하다.
        "omega_min_speed_mps": 0.15,
        # 프로세스 잡음 [m/s²]. CV 는 등속 가정을 좁게 잡아야 선회가 CT 쪽 우도로 간다 (docs/VERIFICATION 'CT 모드 확률').
        "sigma_a_cv": 0.8,
        "sigma_a_ct": 1.2,
        # 모드 체류 시간 [s] (CV, CT). 프레임당 전이확률 = dt/체류시간 이라 프레임률과 무관하다. 정상분포 p_ct = 1/3.
        # 예전의 프레임당 고정 행렬은 30 fps 에서 모드 기억이 0.3~0.6 s 뿐이라 우도가 쌓이기 전에 정상분포로 되돌아갔다.
        "mode_sojourn_sec": [10.0, 5.0],
    },
    "reliability": {
        "min_reliability": 0.05,
        "mahalanobis_threshold_3d": 11.34,  # chi-square df=3, p≈0.99
        "mahalanobis_threshold_2d": 9.21,   # chi-square df=2, p≈0.99
        "base_R_rgbd_diag": [0.15**2, 0.15**2, 0.25**2],
        "base_R_bearing_diag": [0.03**2, 0.03**2],
        "base_R_gps_diag": [2.0**2, 2.0**2, 3.0**2],
        # ESP32 가 주는 상대 속도의 기본 측정 잡음. GPS 속도해는 위치해보다 정확하지만 m/s 단위 오차가 남는다.
        "base_R_vel_diag": [0.30**2, 0.30**2, 0.40**2],
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
        # 최소 이격: 리더까지의 거리가 이 값 아래로 들어가면 접근 성분을 잘라내고 침범량에 비례해 물러난다.
        # 선회 중 최근접 1.59m 가 관측돼(VERIFICATION 남은 결함) 목표 3.0m 와 충돌 사이에 바닥을 둔다.
        "min_separation_m": 2.0,
        "min_separation_kp": 0.6,   # 침범 1m 당 후퇴 속도 [m/s]. 거리 1.42m 위에서는 P 항이 더 강해 이 항은 놀고 있다(설계상 정상).
        # 후퇴만으로는 못 막는 구간이 있다: 리더가 MAX_VX(0.35 m/s)보다 빠르게 다가오면 정면으로는 절대 벗어날 수 없다.
        # 이 거리 아래에서는 시선에 수직인 방향으로 비켜선다 — 느린 기체가 쓸 수 있는 유일한 회피다.
        "evade_radius_m": 1.5,
        "evade_speed_mps": 0.22,    # MAX_VY 와 같은 값 (측면 최대)
        # 반경 안쪽 이 거리만에 최대 측면 속도에 도달한다. (반경-거리)/반경 으로 훑으면 가장 위험한
        # 근거리에서 회피가 가장 약해진다 — SITL 실측 0.65m 에서 명령이 0.125 m/s 뿐이었다.
        "evade_ramp_m": 0.3,
        # 회피 방향을 정할 때 쓰는 좌우 오차 데드밴드. 리더가 정면이면 right 부호가 추정 잡음으로 뒤집혀
        # 회피 방향이 매 프레임 바뀌고 서로 상쇄된다 — SITL 실측에서 0.22 m/s 를 명령하고도 기체는 0.01 m/s 였다.
        # 이 값보다 작으면 부호를 보지 않고 한쪽으로 통일하며, 한 번 정한 방향은 반경을 벗어날 때까지 유지한다.
        "evade_side_deadband_m": 0.30,
        # 리더 속도 피드포워드: cmd = KFF·v_leader + Kp·e + Kd·v_rel.
        # 없으면 정상상태 거리 오차 = v_leader/Kp (0.3m/s→1.4m, 1m/s→4.5m 로 깊이창 10m 밖으로 밀려 소실).
        # KFF<1 로 두는 이유는 EKF 속도 지연을 통한 자기 속도 양성 되먹임의 이득 여유 (main.leader_velocity_ff 주석).
        "leader_vel_ff_gain": 0.8,
        # 피드포워드 1차 저역통과. 2026-09-18 에는 2.0s 였다 — 자기 속도 + 상대 속도로 만든 리더 속도의 지연 불일치가
        # 양성 되먹임이라 그 경로를 P+D 공진 아래로 내려야 했다. 2026-09-19 추정기가 리더 절대 속도를 직접 추정하면서
        # (자기 속도는 예측 입력) 그 경로가 사라졌고, 긴 저역통과는 오히려 FF 를 P 응답보다 늦게 만들어 스트링 안정을
        # 깬다(τ 2.0s 에서 |Γ| 1.14, 0.1s 에서 1.00). 0.1s 는 호버 잡음을 누르는 최소값 (명령 σ 0.03 m/s).
        "leader_vel_ff_tau_sec": 0.1,
        # 자기 속도 감쇠 −KV·v_self (시간간격 정책, h = KV/Kp ≈ 0.7s). 등간격 정책은 선행 기체 정보만으로는 스트링
        # 안정이 안 된다 — 이 항이 |Γ| 피크를 1.14 → 1.00 으로 내린다. 대가: 정상상태 이격 0.3 m/s 에서 0.45 → 0.53 m
        # (Kp 0.22 → 0.30 로 일부 되샀다). docs/STABILITY_MARGINS.md 7절.
        "self_vel_damping": 0.2,
        # 소프트 데드존: |v| ≤ 이 값이면 0, 그 위는 크기에서 이 값을 뺀다(기울기 1). 예전 0.10 램프(2배까지 선형)는 국소 기울기가
        # 3 이라 리더 0.1~0.2 m/s 에서 실효 KFF 가 2.4 가 됐다. 빼는 만큼 정상상태 오차가 KFF·DB/Kp 늘므로(0.10 → +0.36m)
        # 폭을 0.05 로 줄였다. 호버 잡음은 위의 2.0s 저역통과가 평균내므로 데드존은 바이어스만 막으면 된다.
        "leader_vel_ff_deadband_mps": 0.05,
    },
    "logger": {
        "enabled": True,
        "log_dir": "logs",
    },
}
