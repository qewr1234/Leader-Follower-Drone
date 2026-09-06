"""
utils_geometry.py — 좌표/ROI/기하 유틸
"""

import numpy as np


def clamp(x, lo, hi):
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
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    union = bbox_area(a) + bbox_area(b) - inter
    if union <= 0:
        return 0.0
    return inter / union


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


def pixel_to_camera(u, v, depth_m, intrinsics):
    fx = intrinsics.get("fx", 384.0)
    fy = intrinsics.get("fy", 384.0)
    cx = intrinsics.get("ppx", intrinsics.get("cx", 320.0))
    cy = intrinsics.get("ppy", intrinsics.get("cy", 240.0))
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return np.array([x, y, z], dtype=float)


def pixel_to_bearing(u, v, intrinsics):
    fx = intrinsics.get("fx", 384.0)
    fy = intrinsics.get("fy", 384.0)
    cx = intrinsics.get("ppx", intrinsics.get("cx", 320.0))
    cy = intrinsics.get("ppy", intrinsics.get("cy", 240.0))
    bx = (u - cx) / fx
    by = (v - cy) / fy
    return np.array([bx, by], dtype=float)


def camera_to_pixel(point_3d, intrinsics):
    x, y, z = point_3d[:3]
    if z <= 1e-6:
        return None
    fx = intrinsics.get("fx", 384.0)
    fy = intrinsics.get("fy", 384.0)
    cx = intrinsics.get("ppx", intrinsics.get("cx", 320.0))
    cy = intrinsics.get("ppy", intrinsics.get("cy", 240.0))
    u = fx * x / z + cx
    v = fy * y / z + cy
    return np.array([u, v], dtype=float)
