"""
scheduler.py — risk-bound MARS-IMM perception scheduler

핵심:
- IMM state covariance P를 영상 평면으로 투영
- projected covariance ellipse 기반으로 ROI 크기 결정
- IMM mode probability와 영상 uncertainty로 detector 주기 결정
- heuristic gain 중심이 아니라 miss-risk 기반 설계로 보이게 만듦
"""

import numpy as np

from config import CONFIG
from utils_geometry import camera_to_pixel, make_square_roi


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

        p_cv = float(mu[0]) if len(mu) > 0 else 1.0
        p_ct = float(mu[1]) if len(mu) > 1 else 0.0

        Sigma_uv = self._project_covariance_to_image(x, P, intrinsics)
        img_unc = float(np.sqrt(max(np.trace(Sigma_uv), 0.0)))

        roi_size = self._risk_bound_roi_size(
            Sigma_uv=Sigma_uv,
            p_ct=p_ct,
            lost_count=lost_count,
            cfg=cfg,
        )

        roi = make_square_roi(uv[0], uv[1], roi_size, W, H)

        detect_every = self._choose_detect_period(
            img_unc=img_unc,
            p_cv=p_cv,
            p_ct=p_ct,
            lost_count=lost_count,
            cfg=cfg,
        )

        run_detector = (self.frame_idx % max(1, detect_every) == 0)

        return {
            "run_detector": run_detector,
            "use_full_frame": False,
            "roi": roi,
            "detect_every": int(detect_every),
            "reason": "risk_bound_roi",
            "p_cv": p_cv,
            "p_ct": p_ct,
            "img_unc": img_unc,
            "roi_size": int(roi_size),
            "lost_count": lost_count,
        }

    def _full_frame(self, reason):
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
        """
        3D position covariance P[:3,:3]를 image plane covariance로 투영.

        camera coordinate:
            x = [X, Y, Z, ...]
            u = fx * X / Z + cx
            v = fy * Y / Z + cy
        """
        X, Y, Z = float(x[0]), float(x[1]), float(x[2])
        Z = max(Z, 1e-4)

        fx = intrinsics.get("fx", 384.0)
        fy = intrinsics.get("fy", 384.0)

        J = np.array(
            [
                [fx / Z, 0.0, -fx * X / (Z * Z)],
                [0.0, fy / Z, -fy * Y / (Z * Z)],
            ],
            dtype=float,
        )

        P_pos = P[:3, :3]

        # 수치 안정성용
        P_pos = 0.5 * (P_pos + P_pos.T)
        Sigma_uv = J @ P_pos @ J.T

        # detector bbox center noise 최소값 추가
        min_pixel_var = 4.0 ** 2
        Sigma_uv += np.eye(2) * min_pixel_var

        return 0.5 * (Sigma_uv + Sigma_uv.T)

    @staticmethod
    def _risk_bound_roi_size(Sigma_uv, p_ct, lost_count, cfg):
        """
        2D Gaussian ellipse의 chi-square bound를 이용해 ROI 크기 결정.

        chi-square df=2:
            95% ≈ 5.99
            99% ≈ 9.21
            99.7% ≈ 11.83

        급기동 p_ct가 높거나 lost_count가 있으면 더 보수적으로 키움.
        """
        chi2_2d_99 = 9.21

        eigvals = np.linalg.eigvalsh(Sigma_uv)
        max_std = float(np.sqrt(max(np.max(eigvals), 1e-6)))

        # ellipse radius를 square ROI 한 변으로 변환
        radius = np.sqrt(chi2_2d_99) * max_std

        # 실제 target bbox 여유분
        base_margin = float(cfg.get("base_roi_size", 320)) * 0.35

        # maneuver/lost 상황 보수적 확장
        maneuver_margin = float(cfg.get("base_roi_size", 320)) * 0.35 * float(p_ct)
        lost_margin = float(cfg.get("base_roi_size", 320)) * 0.15 * float(lost_count)

        size = 2.0 * radius + base_margin + maneuver_margin + lost_margin

        return int(
            np.clip(
                size,
                cfg["min_roi_size"],
                cfg["max_roi_size"],
            )
        )

    @staticmethod
    def _choose_detect_period(img_unc, p_cv, p_ct, lost_count, cfg):
        """
        detector 호출 주기 결정.

        원리:
        - 영상 plane uncertainty가 작고 CV mode가 강하면 detector를 덜 자주 호출
        - CT mode / maneuver 가능성이 크면 매 프레임 호출
        - lost_count가 있으면 recovery 우선
        """
        if lost_count > 0:
            return 1

        if p_ct > 0.50:
            return cfg["maneuver_detect_every"]

        # 영상 불확실성이 크면 detector를 자주 호출
        if img_unc > 45.0:
            return 1

        if p_cv > 0.75 and img_unc < 25.0:
            return cfg["hover_detect_every"]

        return cfg["normal_detect_every"]
