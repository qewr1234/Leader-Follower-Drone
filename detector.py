"""
detector.py — YOLO11n/TensorRT 탐지 모듈

- ROI 입력 지원
- detect()는 원본 이미지 좌표 기준 bbox 반환
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
    # 고정 크기 TensorRT 엔진은 config의 imgsz와 같은 크기로 내보내야 한다. 다르면
    # ultralytics가 엔진 크기로 덮어써서 설정한 imgsz가 조용히 무시된다.
    print(f"[YOLO] TensorRT 생성 예: yolo export model={PT_MODEL} format=engine "
          f"imgsz={CONFIG['detector']['imgsz']} half=True")
    return YOLO(PT_MODEL, task="detect")


class YoloDetector:
    def __init__(self, model=None, conf_thres=CONF_THRES, target_class_name=TARGET_CLASS_NAME):
        self.model = model if model is not None else load_model()
        self.conf_thres = conf_thres
        self.target_class_name = target_class_name

        # 모델이 이 클래스를 모르면 detect()가 영원히 빈 리스트를 반환한다.
        # 조용히 실패하면 카메라도 YOLO도 FPS도 정상으로 보이는 채 검출만 0이라
        # 현장에서 원인을 찾는 데 시간을 통째로 버린다.
        names = getattr(self.model, "names", None)
        if names:
            available = list(names.values()) if isinstance(names, dict) else list(names)
            if target_class_name not in available:
                print("=" * 78)
                print(f"[YOLO] !! target_class_name='{target_class_name}' 이(가) 모델에 없습니다.")
                print(f"[YOLO]    모델 클래스: {available}")
                print(f"[YOLO]    이대로 두면 검출이 영원히 0건입니다. "
                      f"config.py의 target_class_name을 고치세요.")
                print("=" * 78)
            else:
                print(f"[YOLO] target_class_name='{target_class_name}' 확인됨 "
                      f"(모델 클래스 {len(available)}종)")

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

