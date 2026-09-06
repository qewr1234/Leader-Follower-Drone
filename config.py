"""
config.py — MARS-IMM 설정값

MARS-IMM:
Mode-Aware Reliability-Scheduled IMM Tracking
"""

CONFIG = {
		"detector": {
		    "pt_model": "/home/dsl/DRONE/leader_drone_yolo11n.pt",
		    "trt_model": "/home/dsl/DRONE/leader_drone_yolo11n.engine",
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
        "depth_scale_fallback": 0.001,
    },
    "measurement": {
        "bbox_inner_ratio": 0.55,
        "min_depth_valid_count": 20,
        "min_depth_valid_ratio": 0.15,
        "max_depth_mad": 0.50,
    },
    "imm": {
        "max_coast_sec": 2.0,
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
        "hover_detect_every": 3,
        "normal_detect_every": 2,
        "maneuver_detect_every": 1,
        "uncertainty_gain": 0.8,
        "maneuver_gain": 1.2,
        "lost_gain": 0.35,
    },
    "controller": {
        "hold_after_lost": 5.0,
        "test_interval": 0.10,
        "test_duration": 0.35,
        "uncertainty_slowdown_trace": 4.0,
    },
    "logger": {
        "enabled": True,
        "log_dir": "logs",
    },
}
