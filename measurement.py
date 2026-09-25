"""
measurement.py — 트랙(bbox) + depth 를 IMM-EKF 측정으로 변환
"""

import numpy as np

from config import CONFIG
from utils_geometry import bbox_center, pixel_to_camera, pixel_to_bearing

# 이보다 큰 깊이 ROI 는 2배 서브샘플해 median/MAD 를 구한다. 근접 시(bbox 400px)
# float32 변환 + 정렬이 프레임당 수 ms 를 쓰는데, 중앙값은 1/4 표본으로도 같다.
SUBSAMPLE_ABOVE_PX = 20000

# 가장 가까운 깊이 무리(nearest mode) 선택. 속이 빈 드론 기체를 옆에서 보면 bbox 안쪽 55 % 영역의 절반 이상이 배경이라
# 단순 중앙값이 배경(예: 8 m 지면) 을 잡는다 — 리더가 3 m 인데 "멀다" 고 판단해 전속 전진한다. 배경이 사람처럼 한 덩어리면
# 두 방법은 같은 값이다. DS-KCF(Hannuna·Camplani et al. 2016) 가 같은 이유로 ROI 깊이 히스토그램의 가장 가까운 유의미한
# 모드를 표적으로 잡는다. 빈 폭 0.10 m, 무리 시작 문턱 max(5 % 유효 화소, 30 px), 이어지는 빈은 1 % 이상이면 같은 무리.
NEAREST_MODE_BIN_M = 0.10
NEAREST_MODE_START_FRAC = 0.05
NEAREST_MODE_START_MIN_PX = 30
NEAREST_MODE_CONT_FRAC = 0.01

# 전방 여유(forward_clearance): 프레임 중앙 50 % 영역을 4 배 서브샘플해, 가까운 쪽에서 CLEARANCE_CLOSE_SAMPLES 번째 표본의
# 깊이(= 원본 약 640 px, 25×25 px 이상의 덩어리가 그보다 가깝다). 백분위(5 %) 로 하면 1.2 m 의 350 급 기체 몸통(≈50×20 px,
# 중앙 영역의 1.3 %) 을 놓친다. 프로펠러·랜딩기어가 가장자리에 보여도 중앙만 보므로 걸리지 않는다.
CLEARANCE_CENTER_FRAC = 0.5
CLEARANCE_SUBSAMPLE = 4
CLEARANCE_MIN_VALID_PX = 50
CLEARANCE_CLOSE_SAMPLES = 40


def nearest_mode_range(valid_depths_m, depth_min_m, depth_max_m, bin_m=NEAREST_MODE_BIN_M):
    """유효 깊이 표본에서 가장 가까운 무리를 고른다. 반환 (그 무리의 표본 배열) — 무리를 못 찾으면 전체(기존 중앙값과 동일)."""
    d = np.asarray(valid_depths_m, dtype=np.float32)
    n = int(d.size)
    if n == 0:
        return d
    edges = np.arange(float(depth_min_m), float(depth_max_m) + bin_m, bin_m, dtype=np.float32)
    counts, _ = np.histogram(d, bins=edges)
    start_thr = max(int(np.ceil(NEAREST_MODE_START_FRAC * n)), NEAREST_MODE_START_MIN_PX)
    cont_thr = max(int(np.ceil(NEAREST_MODE_CONT_FRAC * n)), 1)
    starts = np.flatnonzero(counts >= start_thr)
    if starts.size == 0:
        return d
    lo = int(starts[0])
    hi = lo
    while hi + 1 < counts.size and counts[hi + 1] >= cont_thr:
        hi += 1
    sel = d[(d >= edges[lo]) & (d < edges[hi + 1])]
    return sel if sel.size else d


class MeasurementBuilder:
    def __init__(self, intrinsics, depth_scale=0.001):
        self.intrinsics = intrinsics
        self.depth_scale = depth_scale
        self.depth_min_m = CONFIG["camera"]["depth_min_m"]
        self.depth_max_m = CONFIG["camera"]["depth_max_m"]
        self.inner_ratio = CONFIG["measurement"]["bbox_inner_ratio"]
        self.min_valid = CONFIG["measurement"]["min_depth_valid_count"]

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
            # bbox 가 프레임 가장자리에 닿으면 중심이 표적 중심이 아니다(사람은 3 m 에서 세로 348/480 px — pitch 6° 면 잘린다).
            "truncated": bool(track.get("truncated", False)),
        }

    @staticmethod
    def bbox_truncated(bbox, width, height):
        x1, y1, x2, y2 = bbox
        return bool(x1 <= 0 or y1 <= 0 or x2 >= width - 1 or y2 >= height - 1)

    def build_rgbd(self, track, depth_image):
        if track is None or track.get("is_lost", False) or depth_image is None:
            return None
        H, W = depth_image.shape[:2]
        track = dict(track, truncated=self.bbox_truncated(track["bbox"], W, H))
        stats = self._depth_stats(depth_image, track["bbox"])
        if stats["depth_m"] is None:
            return None
        u, v = bbox_center(track["bbox"])
        m = self._base(track, "rgbd", pixel_to_camera(u, v, stats["depth_m"], self.intrinsics), u, v)
        m["bearing"] = pixel_to_bearing(u, v, self.intrinsics)
        m.update(stats)
        return m

    def build_bearing(self, track):
        if track is None or track.get("is_lost", False):
            return None
        u, v = bbox_center(track["bbox"])
        return self._base(track, "bearing", pixel_to_bearing(u, v, self.intrinsics), u, v)

    @staticmethod
    def _depth_result(depth_m=None, valid_ratio=0.0, mad=None, count=0):
        return {"depth_m": depth_m, "depth_valid_ratio": valid_ratio, "depth_mad": mad, "depth_valid_count": count}

    def _depth_stats(self, depth_image, bbox):
        """bbox 중앙 inner_ratio 영역의 유효 깊이 median / MAD."""
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

        # 가장 가까운 무리 안에서 median/MAD — 무리가 하나뿐이면(단단한 표적) 전체 중앙값과 같다.
        sel = nearest_mode_range(valid, self.depth_min_m, self.depth_max_m)
        med = float(np.median(sel))
        mad = float(np.median(np.abs(sel - med)))
        return self._depth_result(med, valid_ratio, mad, int(valid.size))

    def forward_clearance(self, depth_image):
        """프레임 중앙 영역에서 가장 가까운 유효 깊이(5 번째 백분위, m). 유효 화소가 모자라면 None (모름 = 차단하지 않음)."""
        if depth_image is None:
            return None
        H, W = depth_image.shape[:2]
        fy, fx = int(H * (1 - CLEARANCE_CENTER_FRAC) / 2), int(W * (1 - CLEARANCE_CENTER_FRAC) / 2)
        roi = depth_image[fy:H - fy:CLEARANCE_SUBSAMPLE, fx:W - fx:CLEARANCE_SUBSAMPLE]
        if roi.size == 0:
            return None
        d = roi.astype(np.float32) * self.depth_scale
        valid = d[(d >= self.depth_min_m) & (d <= self.depth_max_m)]
        if valid.size < CLEARANCE_MIN_VALID_PX:
            return None
        k = min(CLEARANCE_CLOSE_SAMPLES, int(valid.size)) - 1
        return float(np.partition(valid, k)[k])
