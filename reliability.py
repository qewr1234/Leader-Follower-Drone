"""
reliability.py — 센서 신뢰도, adaptive R, Mahalanobis gating
"""

import numpy as np

from config import CONFIG


class ReliabilityEstimator:
    def __init__(self):
        self.min_r = CONFIG["reliability"]["min_reliability"]
        self.R_rgbd0 = np.diag(CONFIG["reliability"]["base_R_rgbd_diag"])
        self.R_bearing0 = np.diag(CONFIG["reliability"]["base_R_bearing_diag"])
        self.R_gps0 = np.diag(CONFIG["reliability"]["base_R_gps_diag"])

    def vision_reliability(self, track_or_meas):
        if track_or_meas is None:
            return 0.0

        conf = float(track_or_meas.get("conf", 0.0))
        lost_count = int(track_or_meas.get("lost_count", 0))
        age = int(track_or_meas.get("track_age", track_or_meas.get("age", 0)))

        r_conf = np.clip((conf - 0.15) / 0.75, 0.0, 1.0)
        r_lost = float(np.exp(-0.35 * lost_count))
        r_age = np.clip(age / 5.0, 0.3, 1.0)

        # detector를 일부러 실행하지 않은 프레임은
        # 마지막 bbox 기반 measurement이므로 신뢰도를 낮춘다.
        if track_or_meas.get("detector_skipped", False):
            r_skip = 0.55
        else:
            r_skip = 1.0
        # bbox 가 프레임 가장자리에 잘리면 중심(→ bearing·측면 위치) 이 치우친다. 깊이는 안쪽 영역이라 살아 있으므로
        # 버리지 않고 R 만 키운다.
        r_trunc = 0.6 if track_or_meas.get("truncated", False) else 1.0

        return float(np.clip(r_conf * r_lost * r_age * r_skip * r_trunc, 0.0, 1.0))

    def depth_reliability(self, meas):
        if meas is None or meas.get("depth_m") is None:
            return 0.0

        valid_ratio = float(meas.get("depth_valid_ratio", 0.0))
        mad = meas.get("depth_mad", None)

        if mad is None:
            return 0.0

        # 깊이 outlier가 과반이면 median이 outlier에 앉는데도 신뢰도가 높게 나와
        # 제어가 포화된 전속 후진으로 갔다. 산포가 한계를 넘으면 측정을 버린다.
        if float(mad) > float(CONFIG["measurement"]["max_depth_mad"]):
            return 0.0

        r_valid = np.clip(
            valid_ratio / max(CONFIG["measurement"]["min_depth_valid_ratio"], 1e-6),
            0.0,
            1.0,
        )
        r_mad = 1.0 / (1.0 + 8.0 * float(mad))

        return float(np.clip(r_valid * r_mad, 0.0, 1.0))

    # 스테레오 깊이 오차는 z² 에 비례한다 (D435: 기선 50 mm, f≈385 px, 서브픽셀 ≈0.08 → σ_z ≈ 0.006·z²: 3 m 0.05, 8 m 0.4 m).
    # 기본 σ_z 0.25 는 6.5 m 까지는 이미 보수적이고 그 너머는 과신이므로 z 로 키운다.
    STEREO_SIGMA_Z_COEF = 0.006

    def make_R_rgbd(self, r_vision, r_depth, depth_m=None):
        r = max(float(r_vision) * float(r_depth), self.min_r)
        R = self.R_rgbd0 / r
        if depth_m is not None and np.isfinite(depth_m):
            sig_z = max(float(np.sqrt(self.R_rgbd0[2, 2])), self.STEREO_SIGMA_Z_COEF * float(depth_m) ** 2)
            R[2, 2] = max(R[2, 2], sig_z * sig_z / r)
        return R

    def make_R_bearing(self, r_vision):
        r = max(float(r_vision), self.min_r)
        return self.R_bearing0 / r

    def make_R_gps(self, r_gps):
        r = max(float(r_gps), self.min_r)
        return self.R_gps0 / r

    @staticmethod
    def mahalanobis_distance(innovation, S):
        try:
            return float(innovation.T @ np.linalg.inv(S) @ innovation)
        except np.linalg.LinAlgError:
            return float("inf")

    def gate_position3d(self, ekf, z, R):
        y, S = ekf.innovation_position3d(z, R)
        d2 = self.mahalanobis_distance(y, S)
        return d2 <= CONFIG["reliability"]["mahalanobis_threshold_3d"], d2

    def gate_bearing2d(self, ekf, z, R):
        y, S = ekf.innovation_bearing2d(z, R)
        d2 = self.mahalanobis_distance(y, S)
        return d2 <= CONFIG["reliability"]["mahalanobis_threshold_2d"], d2
