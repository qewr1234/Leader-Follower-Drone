"""
detector.py — YOLO11n 탐지 모듈 (TensorRT 엔진 우선, .pt 폴백)

- .engine 이 있으면 TensorRT, 없으면 .pt. 어느 쪽이 도는지는 warmup() 뒤의
  `[YOLO] backend=... device=... fp16=... imgsz=...` 한 줄로 확인한다.
- detect(image, roi) 는 원본 이미지 좌표의 bbox 목록을 돌려준다.
"""

import json
import os
import struct
import time

import numpy as np
from ultralytics import YOLO

from config import CONFIG
from utils_geometry import clip_bbox, bbox_area

PT_MODEL = CONFIG["detector"]["pt_model"]
TRT_MODEL = CONFIG["detector"]["trt_model"]
CONF_THRES = CONFIG["detector"]["conf_thres"]
IOU_THRES = CONFIG["detector"]["iou_thres"]
TARGET_CLASS_NAME = CONFIG["detector"]["target_class_name"]
IMGSZ = CONFIG["detector"]["imgsz"]


def _engine_meta(path):
    """ultralytics 가 export 한 .engine 은 파일 앞에 4바이트 LE 길이 + JSON 메타(imgsz, args.dynamic ...)를 둔다.
    (imgsz, dynamic) — 읽을 수 없으면 (None, None). ultralytics 내부 객체에 기대지 않으려고 파일에서 직접 읽는다."""
    try:
        with open(path, "rb") as f:
            n = struct.unpack("<i", f.read(4))[0]
            if not 0 < n < 1_000_000:
                return None, None
            meta = json.loads(f.read(n).decode("utf-8"))
        imgsz = meta.get("imgsz")
        if isinstance(imgsz, (list, tuple)):
            imgsz = imgsz[0]
        args = meta.get("args") if isinstance(meta.get("args"), dict) else {}
        return (int(imgsz) if imgsz else None), bool(args.get("dynamic", meta.get("dynamic", False)))
    except Exception:
        return None, None


def _torch_device():
    """.pt 폴백용. torch 가 없으면(테스트 스텁 환경) None → ultralytics 기본 동작."""
    try:
        import torch
    except ImportError:
        return None
    return 0 if torch.cuda.is_available() else "cpu"


def load_model():
    if os.path.exists(TRT_MODEL):
        print(f"[YOLO] TensorRT engine load: {TRT_MODEL}")
        return YOLO(TRT_MODEL, task="detect")
    print(f"[YOLO] !! 엔진 없음 ({TRT_MODEL}) — PyTorch 모델로 폴백: {PT_MODEL}")
    print(f"[YOLO]    같은 Jetson 에서 생성: yolo export model={PT_MODEL} format=engine imgsz={IMGSZ} half=True")
    return YOLO(PT_MODEL, task="detect")


def _postprocess(data, offset_x, offset_y, W, H, names, target_class_name):
    """boxes.data 행렬 [[x1,y1,x2,y2,conf,cls],...] → 원본 좌표 검출 dict 목록 (conf, area 내림차순)."""
    detections = []
    for x1, y1, x2, y2, conf, cls in np.asarray(data, dtype=float):
        cls_id = int(cls)
        name = names[cls_id] if names is not None else str(cls_id)
        if target_class_name is not None and name != target_class_name:
            continue
        if x2 - x1 < 2.0 or y2 - y1 < 2.0:
            continue                        # 퇴화 검출(폭·높이 < 2 px). clip_bbox 가 1 px 로 넓혀 살리기 전에 버린다
        bbox = clip_bbox((int(x1) + offset_x, int(y1) + offset_y, int(x2) + offset_x, int(y2) + offset_y), W, H)
        detections.append({"bbox": bbox, "conf": float(conf), "cls": cls_id, "name": name, "area": bbox_area(bbox)})
    detections.sort(key=lambda d: (d["conf"], d["area"]), reverse=True)
    return detections


class YoloDetector:
    def __init__(self, model=None, conf_thres=CONF_THRES, target_class_name=TARGET_CLASS_NAME):
        self.model = model if model is not None else load_model()
        self.conf_thres = conf_thres
        self.target_class_name = target_class_name
        self.is_engine = model is None and os.path.exists(TRT_MODEL)
        self.imgsz = IMGSZ
        self.predict_kwargs = {}

        if self.is_engine:
            # 고정 크기 엔진에 다른 imgsz 를 넘기면 ultralytics 가 AssertionError/TRT 오류로 죽는다.
            # 엔진 헤더의 값을 실효 imgsz 로 쓴다.
            eng_imgsz, dynamic = _engine_meta(TRT_MODEL)
            if eng_imgsz and not dynamic and eng_imgsz != self.imgsz:
                print("=" * 78)
                print(f"[YOLO] !! 엔진 imgsz={eng_imgsz} 가 config imgsz={self.imgsz} 와 다릅니다. "
                      f"엔진 값을 씁니다 — config 를 맞추거나 imgsz={self.imgsz} 로 다시 export 하세요.")
                print("=" * 78)
                self.imgsz = eng_imgsz
        elif model is None:
            dev = _torch_device()
            if dev is not None:
                self.predict_kwargs = {"device": dev, "half": dev != "cpu"}
            if dev == "cpu":
                print("=" * 78)
                print("[YOLO] !! CUDA 없음 — CPU FP32 추론입니다. Jetson 이면 NVIDIA 의 CUDA torch 휠을 설치하세요.")
                print("[YOLO]    이 상태로는 한 자리수 FPS 라 비행에 쓸 수 없습니다.")
                print("=" * 78)

        # 모델이 이 클래스를 모르면 detect()가 영원히 빈 리스트를 반환한다. 조용히 실패하면
        # 카메라도 YOLO도 FPS도 정상으로 보이는 채 검출만 0이라 현장에서 원인을 찾느라 시간을 버린다.
        self.names = getattr(self.model, "names", None)
        self.target_cls_id = None
        if self.names:
            items = self.names.items() if isinstance(self.names, dict) else enumerate(self.names)
            ids = [i for i, n in items if n == target_class_name]
            if ids:
                self.target_cls_id = int(ids[0])
                print(f"[YOLO] target_class_name='{target_class_name}' 확인됨 (모델 클래스 {len(self.names)}종)")
            else:
                print("=" * 78)
                print(f"[YOLO] !! target_class_name='{target_class_name}' 이(가) 모델에 없습니다.")
                print(f"[YOLO]    모델 클래스: {list(self.names.values()) if isinstance(self.names, dict) else list(self.names)}")
                print(f"[YOLO]    이대로 두면 검출이 영원히 0건입니다. config.py의 target_class_name을 고치세요.")
                print("=" * 78)

        # 매 호출 kwargs 로 넘긴다. model.overrides 에 넣으면 ultralytics Model.predict 가
        # {**overrides, **custom(conf=0.25), **kwargs} 로 병합해 conf 가 0.25 로 덮인다 — kwargs 만 살아남는다.
        self._static = dict(verbose=False, conf=self.conf_thres, iou=IOU_THRES, imgsz=self.imgsz, **self.predict_kwargs)

    def _predict_kwargs(self):
        kw = dict(self._static)
        if self.target_cls_id is not None:
            kw["classes"] = [self.target_cls_id]      # NMS 단계에서 GPU 측 필터
        return kw

    def warmup(self, width, height):
        """엔진 역직렬화·CUDA 컨텍스트·첫 추론 지연을 제어 루프 밖에서 끝낸다. 실제 predict 를 불러야 한다
        (.names 접근만으로는 버전에 따라 predictor 가 남지 않는다). 실패해도 비행 로직과 무관하니 경고만."""
        if not hasattr(self.model, "predict"):
            return
        t0 = time.time()
        try:
            self.model.predict(np.zeros((height, width, 3), dtype=np.uint8), **self._predict_kwargs())
            self.model.predict(np.zeros((320, 320, 3), dtype=np.uint8), **self._predict_kwargs())
        except Exception as exc:
            print(f"[YOLO] !! warm-up 실패: {type(exc).__name__}: {exc}")
            return
        p = getattr(self.model, "predictor", None)
        m = getattr(p, "model", None)
        print(f"[YOLO] warm-up {time.time() - t0:.1f}s | backend={'TensorRT' if self.is_engine else 'PyTorch'} "
              f"device={getattr(p, 'device', '?')} fp16={getattr(m, 'fp16', '?')} imgsz={self.imgsz} conf={self.conf_thres}")

    def detect(self, image, roi=None):
        H, W = image.shape[:2]
        offset_x = offset_y = 0
        infer_img = image
        if roi is not None:
            x1, y1, x2, y2 = clip_bbox(roi, W, H)
            infer_img = image[y1:y2, x1:x2]
            offset_x, offset_y = x1, y1
            if infer_img.size == 0:
                return []

        results = self.model.predict(infer_img, **self._predict_kwargs())
        boxes = results[0].boxes if results else None
        if boxes is None or len(boxes) == 0:
            return []
        data = boxes.data
        data = data.cpu().numpy() if hasattr(data, "cpu") else np.asarray(data)   # GPU→CPU 동기화 1회
        return _postprocess(data, offset_x, offset_y, W, H, self.names, self.target_class_name)
