"""
reliability.py — 센서 신뢰도, adaptive R, Mahalanobis gating
"""

import numpy as np

from config import CONFIG


def _hhv_to_camera_axes(diag3):
    """config 의 [수평, 수평, 수직] 분산을 카메라 축 [right, down, forward] = [수평, 수직, 수평] 으로 옮긴다.
    그대로 쓰면 수직 분산(보통 더 큼)이 전방 거리 축에 붙고 수직 축은 수평 분산을 받는다(이전 감사 #61)."""
    h1, h2, v = (float(x) for x in diag3)
    return np.diag([h1, v, h2])


class ReliabilityEstimator:
    def __init__(self):
        rc = CONFIG["reliability"]
        self.min_r = rc["min_reliability"]
        # RGB-D 기본 R 은 rgbd_range_ref_m 거리에서의 값. R_rgbd0_at(range) 가 거리 배율을 곱한다 (횡 ∝ z, 깊이 ∝ z²; 분산은 제곱).
        self.R_rgbd0 = np.diag(rc["base_R_rgbd_diag"])
        self.range_ref_m = float(rc.get("rgbd_range_ref_m", 3.0))
        self.range_scale_min = float(rc.get("rgbd_range_scale_min", 0.5))
        self.range_scale_max = float(rc.get("rgbd_range_scale_max", 4.0))
        self.R_bearing0 = np.diag(rc["base_R_bearing_diag"])
        self.R_gps0 = _hhv_to_camera_axes(rc["base_R_gps_diag"])
        self.R_vel0 = _hhv_to_camera_axes(rc["base_R_vel_diag"])
        self.min_cluster_ratio = float(CONFIG["measurement"].get("min_depth_cluster_ratio", 0.30))

    def R_rgbd0_at(self, range_m=None):
        """거리 range_m(전방 깊이)에서의 RGB-D 기본 R. None 이면 기준 거리 값. 스테레오 깊이 오차는 거리의 제곱, 화소→횡 위치 오차는
        거리에 비례하므로 σ 에 (z/z_ref)² 와 (z/z_ref) 를 곱한다. 기준 거리(3 m)에서는 정확히 base_R 이라 분석 설계점이 바뀌지 않는다."""
        if range_m is None or not np.isfinite(float(range_m)) or float(range_m) <= 0.0:
            return self.R_rgbd0
        s = min(max(float(range_m) / self.range_ref_m, self.range_scale_min), self.range_scale_max)
        return self.R_rgbd0 * np.array([s * s, s * s, s ** 4])

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

        # 선택한 군집 안의 산포가 한계를 넘으면 측정을 버린다 (군집 분리 뒤라 '배경 과반' 은 여기 오기 전에 걸러진다 — measurement).
        if float(mad) > float(CONFIG["measurement"]["max_depth_mad"]):
            return 0.0

        # 유효 비율: min_depth_valid_ratio 아래는 거부(0), 그 위 2배까지 선형. 예전에는 '램프의 포화점' 이라 거부가 없었다.
        min_ratio = max(float(CONFIG["measurement"]["min_depth_valid_ratio"]), 1e-6)
        if valid_ratio < min_ratio:
            return 0.0
        r_valid = np.clip((valid_ratio - min_ratio) / min_ratio, 0.0, 1.0)
        r_mad = 1.0 / (1.0 + 8.0 * float(mad))
        # 최근접 군집이 유효 화소 중 차지하는 몫. 작으면(배경이 과반인 이중모드 ROI) 그만큼 덜 믿는다 — 버리지는 않는다.
        cluster_ratio = float(meas.get("depth_cluster_ratio", 1.0))
        r_cluster = np.clip(cluster_ratio / max(self.min_cluster_ratio, 1e-6), 0.0, 1.0)

        return float(np.clip(r_valid * r_mad * r_cluster, 0.0, 1.0))

    def make_R_rgbd(self, r_vision, r_depth, range_m=None):
        r = max(float(r_vision) * float(r_depth), self.min_r)
        return self.R_rgbd0_at(range_m) / r

    def make_R_bearing(self, r_vision):
        r = max(float(r_vision), self.min_r)
        return self.R_bearing0 / r

    def make_R_gps(self, r_gps):
        r = max(float(r_gps), self.min_r)
        return self.R_gps0 / r

    def make_R_velocity(self, r_vel):
        r = max(float(r_vel), self.min_r)
        return self.R_vel0 / r

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

    def gate_velocity3d(self, ekf, z, R):
        y, S = ekf.innovation_velocity3d(z, R)
        d2 = self.mahalanobis_distance(y, S)
        return d2 <= CONFIG["reliability"]["mahalanobis_threshold_3d"], d2
