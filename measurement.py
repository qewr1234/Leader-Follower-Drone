"""
measurement.py — bbox/depth/GPS를 필터 measurement로 변환
"""

import numpy as np

from config import CONFIG
from utils_geometry import bbox_center, pixel_to_camera, pixel_to_bearing


class MeasurementBuilder:
    def __init__(self, intrinsics, depth_scale=0.001):
        self.intrinsics = intrinsics
        self.depth_scale = depth_scale
        self.depth_min_m = CONFIG["camera"]["depth_min_m"]
        self.depth_max_m = CONFIG["camera"]["depth_max_m"]
        self.inner_ratio = CONFIG["measurement"]["bbox_inner_ratio"]

    def build_rgbd(self, track, depth_image):
        if track is None or track.get("is_lost", False):
            return None
        if depth_image is None:
            return None

        bbox = track["bbox"]
        depth_stats = self._depth_stats(depth_image, bbox)
        if depth_stats["depth_m"] is None:
            return None

        u, v = bbox_center(bbox)
        z3d = pixel_to_camera(u, v, depth_stats["depth_m"], self.intrinsics)
        bearing = pixel_to_bearing(u, v, self.intrinsics)

        return {
            "type": "rgbd",
            "z": z3d,
            "bearing": bearing,
            "bbox": bbox,
            "cx": u,
            "cy": v,
            "conf": float(track.get("conf", 0.0)),
            "track_age": int(track.get("age", 0)),
            "lost_count": int(track.get("lost_count", 0)),
            **depth_stats,
        }

    def build_bearing(self, track):
        if track is None or track.get("is_lost", False):
            return None
        u, v = bbox_center(track["bbox"])
        bearing = pixel_to_bearing(u, v, self.intrinsics)
        return {
            "type": "bearing",
            "z": bearing,
            "bbox": track["bbox"],
            "cx": u,
            "cy": v,
            "conf": float(track.get("conf", 0.0)),
            "track_age": int(track.get("age", 0)),
            "lost_count": int(track.get("lost_count", 0)),
        }

    def build_gps_relative(self, vehicle_state):
        # 현재 업로드 코드에는 leader GPS가 없으므로, GPS는 quality/gating 확장용 placeholder.
        # leader/follower GPS가 모두 있으면 ENU/NED 상대좌표 변환을 여기에 구현한다.
        return None

    def _depth_stats(self, depth_image, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        H, W = depth_image.shape[:2]
        x1 = max(0, min(W - 1, x1)); x2 = max(0, min(W, x2))
        y1 = max(0, min(H - 1, y1)); y2 = max(0, min(H, y2))
        if x2 <= x1 or y2 <= y1:
            return self._empty_depth()

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        hw = (x2 - x1) * self.inner_ratio / 2.0
        hh = (y2 - y1) * self.inner_ratio / 2.0
        rx1 = int(max(0, cx - hw)); rx2 = int(min(W, cx + hw))
        ry1 = int(max(0, cy - hh)); ry2 = int(min(H, cy + hh))
        roi = depth_image[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            return self._empty_depth()

        depth_m = roi.astype(np.float32) * self.depth_scale
        valid = depth_m[(depth_m >= self.depth_min_m) & (depth_m <= self.depth_max_m)]
        valid_ratio = float(valid.size / max(roi.size, 1))

        if valid.size < CONFIG["measurement"]["min_depth_valid_count"]:
            return {
                "depth_m": None,
                "depth_valid_ratio": valid_ratio,
                "depth_mad": None,
                "depth_iqr": None,
                "depth_valid_count": int(valid.size),
            }

        med = float(np.median(valid))
        mad = float(np.median(np.abs(valid - med)))
        q25, q75 = np.percentile(valid, [25, 75])
        iqr = float(q75 - q25)
        return {
            "depth_m": med,
            "depth_valid_ratio": valid_ratio,
            "depth_mad": mad,
            "depth_iqr": iqr,
            "depth_valid_count": int(valid.size),
        }

    @staticmethod
    def _empty_depth():
        return {
            "depth_m": None,
            "depth_valid_ratio": 0.0,
            "depth_mad": None,
            "depth_iqr": None,
            "depth_valid_count": 0,
        }
