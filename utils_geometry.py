"""
utils_geometry.py — 좌표/ROI/기하 유틸
"""

import numpy as np


def clamp(x, lo, hi):
    """[lo, hi] 로 자른다. NaN 은 0 으로 본 뒤 자른다.

    max(lo, min(hi, nan)) 은 Python 비교 규칙상 hi 를 돌려준다 — 속도 명령에 쓰면 NaN 추정이 '정지' 가 아니라
    '전 축 최대 속도' 로 나간다 (2026-09-24 재현: [0.35, 0.22, -0.12, 0.35]). 0 은 속도·비율·화소 어느 용도에서도
    안전한 쪽이다.
    """
    if x != x:          # NaN
        x = 0.0
    return max(lo, min(hi, x))


def bbox_center(bbox):
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def bbox_area(bbox):
    x1, y1, x2, y2 = bbox
    return max(0, x2 - x1) * max(0, y2 - y1)


def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = bbox_area(a) + bbox_area(b) - inter
    return inter / union if union > 0 else 0.0


def clip_bbox(bbox, width, height):
    x1, y1, x2, y2 = bbox
    x1 = int(clamp(round(x1), 0, width - 1))
    y1 = int(clamp(round(y1), 0, height - 1))
    x2 = int(clamp(round(x2), 0, width - 1))
    y2 = int(clamp(round(y2), 0, height - 1))
    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)
    return (x1, y1, x2, y2)


def make_square_roi(cx, cy, size, width, height):
    half = size / 2.0
    return clip_bbox((cx - half, cy - half, cx + half, cy + half), width, height)


def _intr(intrinsics):
    """(fx, fy, cx, cy). RealSense 는 ppx/ppy, 일반 표기는 cx/cy — 둘 다 받는다."""
    return (
        intrinsics.get("fx", 384.0),
        intrinsics.get("fy", 384.0),
        intrinsics.get("ppx", intrinsics.get("cx", 320.0)),
        intrinsics.get("ppy", intrinsics.get("cy", 240.0)),
    )


def pixel_to_camera(u, v, depth_m, intrinsics):
    fx, fy, cx, cy = _intr(intrinsics)
    return np.array([(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m], dtype=float)


def pixel_to_bearing(u, v, intrinsics):
    fx, fy, cx, cy = _intr(intrinsics)
    return np.array([(u - cx) / fx, (v - cy) / fy], dtype=float)


def camera_to_pixel(point_3d, intrinsics):
    x, y, z = point_3d[:3]
    if z <= 1e-6:
        return None
    fx, fy, cx, cy = _intr(intrinsics)
    return np.array([fx * x / z + cx, fy * y / z + cy], dtype=float)
