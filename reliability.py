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

        return float(np.clip(r_conf * r_lost * r_age * r_skip, 0.0, 1.0))

    def depth_reliability(self, meas):
        if meas is None or meas.get("depth_m") is None:
            return 0.0

        valid_ratio = float(meas.get("depth_valid_ratio", 0.0))
        mad = meas.get("depth_mad", None)

        if mad is None:
            return 0.0

        r_valid = np.clip(
            valid_ratio / max(CONFIG["measurement"]["min_depth_valid_ratio"], 1e-6),
            0.0,
            1.0,
        )
        r_mad = 1.0 / (1.0 + 8.0 * float(mad))

        return float(np.clip(r_valid * r_mad, 0.0, 1.0))

    def gps_reliability(self, gps_data):
        if not gps_data:
            return 0.0

        fix_type = int(gps_data.get("fix_type", 0) or 0)
        sats = int(gps_data.get("satellites_visible", gps_data.get("satellites", 0)) or 0)
        h_acc = gps_data.get("h_acc", None)
        eph = gps_data.get("eph", None)

        r_fix = 1.0 if fix_type >= 3 else 0.0
        r_sat = np.clip((sats - 5) / 7.0, 0.0, 1.0)

        if h_acc is not None and h_acc > 0:
            # MAVLink GPS_RAW_INT h_acc는 mm 단위인 경우가 많음.
            h_m = float(h_acc) / 1000.0 if h_acc > 100 else float(h_acc)
            r_acc = 1.0 / (1.0 + h_m)
        elif eph is not None and eph > 0:
            # eph는 cm scale인 경우가 많음.
            eph_m = float(eph) / 100.0
            r_acc = 1.0 / (1.0 + eph_m)
        else:
            r_acc = 0.4

        return float(np.clip(r_fix * r_sat * r_acc, 0.0, 1.0))

    def make_R_rgbd(self, r_vision, r_depth):
        r = max(float(r_vision) * float(r_depth), self.min_r)
        return self.R_rgbd0 / r

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
