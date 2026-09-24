"""
scheduler.py — risk-bound MARS-IMM perception scheduler

IMM 상태 공분산 P를 영상 평면에 투영해 ROI 크기를 정하고, IMM 모드 확률과 영상 불확실성으로
검출기 호출 주기를 정한다. ROI는 정확도 장치(오검출 억제)이고, FPS 이득은 run_detector=False
인 프레임에서만 난다 — 고정 크기 TensorRT 엔진은 ROI가 작아도 추론 비용이 같다.
"""

import numpy as np

from config import CONFIG
from utils_geometry import camera_to_pixel, make_square_roi

CHI2_2D_99 = 9.21          # chi-square df=2, p=0.99
MIN_PIXEL_VAR = 4.0 ** 2   # detector bbox center noise 하한


class PerceptionScheduler:
    def __init__(self):
        self.frame_idx = 0

    def decide(self, imm_state, image_shape, intrinsics, last_track=None):
        self.frame_idx += 1
        H, W = image_shape[:2]
        cfg = CONFIG["scheduler"]

        if not cfg.get("enable_roi", True):
            return self._full_frame("roi_disabled")
        if imm_state is None or not imm_state.get("initialized", False):
            return self._full_frame("not_initialized")
        if last_track is None:
            return self._full_frame("lost_full_frame")

        lost_count = int(last_track.get("lost_count", 0))
        if lost_count >= cfg["lost_full_frame_threshold"]:
            return self._full_frame("lost_full_frame")
        if self.frame_idx % cfg["full_frame_interval"] == 0:
            return self._full_frame("periodic_full_frame")

        x = np.asarray(imm_state["x"], dtype=float)
        P = np.asarray(imm_state["P"], dtype=float)
        mu = np.asarray(imm_state["mode_probs"], dtype=float)

        uv = camera_to_pixel(x[:3], intrinsics)
        if uv is None or not np.all(np.isfinite(uv)):
            return self._full_frame("bad_projection")
        if not (0.0 <= uv[0] < W and 0.0 <= uv[1] < H):
            return self._full_frame("off_image")      # 추정이 화면 밖 → 1 px 폭 ROI 가 아니라 전체 프레임

        p_cv = float(mu[0]) if len(mu) > 0 else 1.0
        p_ct = float(mu[1]) if len(mu) > 1 else 0.0

        Sigma_uv = self._project_covariance_to_image(x, P, intrinsics)
        img_unc = float(np.sqrt(max(np.trace(Sigma_uv), 0.0)))
        roi_size = self._risk_bound_roi_size(Sigma_uv, p_ct, lost_count, cfg)
        detect_every = self._choose_detect_period(img_unc, p_ct, lost_count, cfg)

        return {
            "run_detector": self.frame_idx % max(1, detect_every) == 0,
            "use_full_frame": False,
            "roi": make_square_roi(uv[0], uv[1], roi_size, W, H),
            "detect_every": int(detect_every),
            "reason": "risk_bound_roi",
            "p_cv": p_cv,
            "p_ct": p_ct,
            "img_unc": img_unc,
            "roi_size": int(roi_size),
            "lost_count": lost_count,
        }

    @staticmethod
    def _full_frame(reason):
        return {
            "run_detector": True,
            "use_full_frame": True,
            "roi": None,
            "detect_every": 1,
            "reason": reason,
            "p_cv": 0.0,
            "p_ct": 0.0,
            "img_unc": 999.0,
            "roi_size": CONFIG["scheduler"]["max_roi_size"],
            "lost_count": 0,
        }

    @staticmethod
    def _project_covariance_to_image(x, P, intrinsics):
        """P[:3,:3] 을 u = fx·X/Z + cx, v = fy·Y/Z + cy 의 야코비안으로 영상 평면에 투영."""
        X, Y, Z = float(x[0]), float(x[1]), float(x[2])
        Z = max(Z, 1e-4)
        fx = intrinsics.get("fx", 384.0)
        fy = intrinsics.get("fy", 384.0)
        J = np.array([[fx / Z, 0.0, -fx * X / (Z * Z)],
                      [0.0, fy / Z, -fy * Y / (Z * Z)]], dtype=float)
        P_pos = 0.5 * (P[:3, :3] + P[:3, :3].T)
        Sigma_uv = J @ P_pos @ J.T + np.eye(2) * MIN_PIXEL_VAR
        return 0.5 * (Sigma_uv + Sigma_uv.T)

    @staticmethod
    def _risk_bound_roi_size(Sigma_uv, p_ct, lost_count, cfg):
        """99% 오차 타원의 반경을 정사각 ROI 한 변으로. 급기동(p_ct)·소실(lost_count)이면 더 키운다."""
        max_std = float(np.sqrt(max(np.max(np.linalg.eigvalsh(Sigma_uv)), 1e-6)))
        radius = np.sqrt(CHI2_2D_99) * max_std
        base = float(cfg.get("base_roi_size", 320))
        size = 2.0 * radius + base * (0.35 + 0.35 * float(p_ct) + 0.15 * float(lost_count))
        return int(np.clip(size, cfg["min_roi_size"], cfg["max_roi_size"]))

    @staticmethod
    def _choose_detect_period(img_unc, p_ct, lost_count, cfg):
        """놓치는 중이거나 기동 중이거나 영상 불확실성이 크면 매 프레임, 아니면 normal_detect_every."""
        if lost_count > 0:
            return 1
        if p_ct > 0.50:
            return cfg["maneuver_detect_every"]
        if img_unc > 45.0:
            return 1
        return cfg["normal_detect_every"]
