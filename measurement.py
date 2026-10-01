"""
measurement.py — 트랙(bbox) + depth 를 IMM-EKF 측정으로 변환
"""

import numpy as np

from config import CONFIG
from utils_geometry import bbox_center, pixel_to_camera, pixel_to_bearing

# 이보다 큰 깊이 ROI 는 2배 서브샘플해 median/MAD 를 구한다. 근접 시(bbox 400px)
# float32 변환 + 정렬이 프레임당 수 ms 를 쓰는데, 중앙값은 1/4 표본으로도 같다.
SUBSAMPLE_ABOVE_PX = 20000


class MeasurementBuilder:
    def __init__(self, intrinsics, depth_scale=0.001):
        self.intrinsics = intrinsics
        self.depth_scale = depth_scale
        self.depth_min_m = CONFIG["camera"]["depth_min_m"]
        self.depth_max_m = CONFIG["camera"]["depth_max_m"]
        self.inner_ratio = CONFIG["measurement"]["bbox_inner_ratio"]
        self.min_valid = CONFIG["measurement"]["min_depth_valid_count"]
        self.cluster_gap_m = float(CONFIG["measurement"].get("depth_cluster_gap_m", 0.5))

    @staticmethod
    def _base(track, mtype, z, u, v):
        return {
            "type": mtype,
            "z": z,
            "bbox": track["bbox"],
            "cx": u,
            "cy": v,
            "conf": float(track.get("conf", 0.0)),
            "track_age": int(track.get("age", 0)),
            "lost_count": int(track.get("lost_count", 0)),
            # 검출을 건너뛴 프레임의 bbox 는 마지막 검출 위치다. reliability 가 이 키를 보고
            # 신뢰도를 깎으므로 반드시 넘긴다.
            "detector_skipped": bool(track.get("detector_skipped", False)),
        }

    def build_rgbd(self, track, depth_image):
        if track is None or track.get("is_lost", False) or depth_image is None:
            return None
        stats = self._depth_stats(depth_image, track["bbox"])
        if stats["depth_m"] is None:
            return None
        u, v = bbox_center(track["bbox"])
        m = self._base(track, "rgbd", pixel_to_camera(u, v, stats["depth_m"], self.intrinsics), u, v)
        m.update(stats)
        return m

    def build_bearing(self, track):
        if track is None or track.get("is_lost", False):
            return None
        u, v = bbox_center(track["bbox"])
        return self._base(track, "bearing", pixel_to_bearing(u, v, self.intrinsics), u, v)

    @staticmethod
    def _depth_result(depth_m=None, valid_ratio=0.0, mad=None, count=0, cluster_ratio=1.0, clusters=0):
        return {"depth_m": depth_m, "depth_valid_ratio": valid_ratio, "depth_mad": mad, "depth_valid_count": count,
                # 최근접 군집이 유효 화소에서 차지하는 몫과 틈(gap)으로 나뉜 군집 수. 군집 수 ≥ 2 이면 이중모드 ROI 였다는 뜻.
                "depth_cluster_ratio": cluster_ratio, "depth_clusters": clusters}

    def _nearest_cluster(self, valid_sorted):
        """정렬된 유효 깊이를 cluster_gap_m 보다 큰 틈으로 나눠, min_valid 화소 이상인 **가장 가까운** 군집을 돌려준다 (군집, 군집 수).
        리더는 자기 bbox 안에서 배경보다 앞에 있다. 전체 median 은 배경 화소가 과반이면 배경에 앉는데(이중모드 ROI, 이전 감사 #58)
        MAD 는 산포 검사라 두 모드의 크기가 비슷할 때만 잡는다. 틈으로 나누면 몫과 무관하게 앞쪽 물체를 고른다.
        min_valid 미만의 앞쪽 조각(먼지·프롭 가장자리)은 건너뛴다."""
        if valid_sorted.size == 0:
            return valid_sorted, 0
        gaps = np.flatnonzero(np.diff(valid_sorted) > self.cluster_gap_m)
        bounds = np.concatenate(([0], gaps + 1, [valid_sorted.size]))
        n_clusters = bounds.size - 1
        for i in range(n_clusters):
            seg = valid_sorted[bounds[i]:bounds[i + 1]]
            if seg.size >= self.min_valid:
                return seg, n_clusters
        return valid_sorted[:0], n_clusters

    def _depth_stats(self, depth_image, bbox):
        """bbox 중앙 inner_ratio 영역에서 최근접 깊이 군집의 median / MAD."""
        x1, y1, x2, y2 = map(int, bbox)
        H, W = depth_image.shape[:2]
        x1, x2 = max(0, min(W - 1, x1)), max(0, min(W, x2))
        y1, y2 = max(0, min(H - 1, y1)), max(0, min(H, y2))
        if x2 <= x1 or y2 <= y1:
            return self._depth_result()

        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        hw, hh = (x2 - x1) * self.inner_ratio / 2.0, (y2 - y1) * self.inner_ratio / 2.0
        roi = depth_image[int(max(0, cy - hh)):int(min(H, cy + hh)), int(max(0, cx - hw)):int(min(W, cx + hw))]
        if roi.size == 0:
            return self._depth_result()
        if roi.size > SUBSAMPLE_ABOVE_PX:
            roi = roi[::2, ::2]

        depth_m = roi.astype(np.float32) * self.depth_scale
        valid = depth_m[(depth_m >= self.depth_min_m) & (depth_m <= self.depth_max_m)]
        valid_ratio = float(valid.size / max(roi.size, 1))
        if valid.size < self.min_valid:
            return self._depth_result(None, valid_ratio, None, int(valid.size))

        cluster, n_clusters = self._nearest_cluster(np.sort(valid))
        if cluster.size < self.min_valid:
            return self._depth_result(None, valid_ratio, None, int(valid.size), 0.0, n_clusters)
        med = float(np.median(cluster))
        mad = float(np.median(np.abs(cluster - med)))
        return self._depth_result(med, valid_ratio, mad, int(cluster.size), float(cluster.size / valid.size), n_clusters)
