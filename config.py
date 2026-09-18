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
    },
    "logger": {
        "enabled": True,
        "log_dir": "logs",
    },
}
