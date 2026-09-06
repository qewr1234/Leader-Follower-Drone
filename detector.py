"""
detector.py — YOLO11n/TensorRT 탐지 모듈

변경점:
- ROI 입력 지원
- detect()는 원본 이미지 좌표 기준 bbox 반환
- select_target() 호환 함수 유지
"""

import os
from typing import List, Dict, Optional, Tuple

from ultralytics import YOLO

from config import CONFIG
from utils_geometry import clip_bbox, bbox_area

PT_MODEL = CONFIG["detector"]["pt_model"]
TRT_MODEL = CONFIG["detector"]["trt_model"]
CONF_THRES = CONFIG["detector"]["conf_thres"]
IOU_THRES = CONFIG["detector"]["iou_thres"]
TARGET_CLASS_NAME = CONFIG["detector"]["target_class_name"]


def load_model():
    if os.path.exists(TRT_MODEL):
        print(f"[YOLO] TensorRT engine load: {TRT_MODEL}")
        return YOLO(TRT_MODEL, task="detect")
    print(f"[YOLO] PyTorch model load: {PT_MODEL}")
    print(f"[YOLO] TensorRT 생성 예: yolo export model={PT_MODEL} format=engine imgsz=640 half=True")
    return YOLO(PT_MODEL, task="detect")


class YoloDetector:
    def __init__(self, model=None, conf_thres=CONF_THRES, target_class_name=TARGET_CLASS_NAME):
        self.model = model if model is not None else load_model()
        self.conf_thres = conf_thres
        self.target_class_name = target_class_name

    def detect(self, image, roi: Optional[Tuple[int, int, int, int]] = None) -> List[Dict]:
        H, W = image.shape[:2]
        offset_x, offset_y = 0, 0
        infer_img = image

        if roi is not None:
            x1, y1, x2, y2 = clip_bbox(roi, W, H)
            infer_img = image[y1:y2, x1:x2]
            offset_x, offset_y = x1, y1
            if infer_img.size == 0:
                return []

        results = self.model.predict(
            infer_img,
            verbose=False,
            conf=self.conf_thres,
            iou=IOU_THRES,
            imgsz=CONFIG["detector"]["imgsz"],
        )
        if not results or results[0].boxes is None:
            return []

        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            name = self.model.names[cls_id]
            if name != self.target_class_name:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x1 += offset_x; x2 += offset_x
            y1 += offset_y; y2 += offset_y
            bbox = clip_bbox((x1, y1, x2, y2), W, H)
            detections.append({
                "bbox": bbox,
                "conf": conf,
                "cls": cls_id,
                "name": name,
                "area": bbox_area(bbox),
            })

        detections.sort(key=lambda d: (d["conf"], d["area"]), reverse=True)
        return detections


def select_target(result, model, depth_image=None, conf_thres=CONF_THRES):
    """기존 main.py 호환용. 새 구조에서는 YoloDetector.detect()+tracker 사용 권장."""
    if result.boxes is None:
        return None
    best = None
    best_area = 0
    for box in result.boxes:
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        if model.names[cls_id] != TARGET_CLASS_NAME or conf < conf_thres:
            continue
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        area = max(0, x2 - x1) * max(0, y2 - y1)
        if area > best_area:
            best_area = area
            best = (x1, y1, x2, y2, conf)
    if best is None:
        return None
    x1, y1, x2, y2, conf = best
    H = depth_image.shape[0] if depth_image is not None else 480
    return {
        "bbox": (x1, y1, x2, y2),
        "conf": conf,
        "cx": (x1 + x2) // 2,
        "cy": (y1 + y2) // 2,
        "bh": y2 - y1,
        "h_ratio": (y2 - y1) / max(H, 1),
        "depth_m": None,
    }
