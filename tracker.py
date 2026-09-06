"""
tracker.py — 선두 드론 단일 타겟 트래커

가벼운 IoU 기반 tracker.
나중에 OC-SORT/ByteTrack으로 교체 가능한 인터페이스를 유지한다.
"""

from typing import List, Dict, Optional
from utils_geometry import iou_xyxy, bbox_area


class LeaderTracker:
    def __init__(self, iou_threshold=0.20, max_lost=12):
        self.iou_threshold = iou_threshold
        self.max_lost = max_lost
        self.track = None
        self.next_id = 1

    def reset(self):
        self.track = None

    def predict_only(self):
        """
        detector를 일부러 실행하지 않은 프레임.
        미검출로 처리하지 않고 마지막 track을 유지한다.
        """
        if self.track is None:
            return None

        tr = self.track.copy()
        tr["detector_skipped"] = True
        return tr

    def update(self, detections: List[Dict]) -> Optional[Dict]:
        if self.track is None:
            if not detections:
                return None
            det = self._choose_initial(detections)
            self.track = self._new_track(det)
            return self.track.copy()

        if not detections:
            self.track["lost_count"] += 1
            self.track["is_lost"] = self.track["lost_count"] > 0
            self.track["conf"] *= 0.90
            if self.track["lost_count"] > self.max_lost:
                old = self.track.copy()
                self.track = None
                return old
            return self.track.copy()

        best_det = None
        best_score = -1.0
        prev_bbox = self.track["bbox"]
        for det in detections:
            iou = iou_xyxy(prev_bbox, det["bbox"])
            score = 0.75 * iou + 0.25 * det["conf"]
            if score > best_score:
                best_score = score
                best_det = det

        if best_det is None:
            return self.update([])

        best_iou = iou_xyxy(prev_bbox, best_det["bbox"])
        # IoU가 낮더라도 현재 타겟이 하나뿐인 경우 confidence/area가 높으면 회복 허용
        accept = best_iou >= self.iou_threshold or self.track["lost_count"] > 0

        if accept:
            alpha = 0.65 if self.track["lost_count"] == 0 else 0.85
            self.track["bbox"] = self._smooth_bbox(self.track["bbox"], best_det["bbox"], alpha)
            self.track["conf"] = float(best_det["conf"])
            self.track["area"] = bbox_area(self.track["bbox"])
            self.track["age"] += 1
            self.track["lost_count"] = 0
            self.track["is_lost"] = False
        else:
            self.track["lost_count"] += 1
            self.track["is_lost"] = True

        return self.track.copy()

    def _choose_initial(self, detections):
        return max(detections, key=lambda d: d.get("area", 0) * max(d.get("conf", 0.0), 0.01))

    def _new_track(self, det):
        tr = det.copy()
        tr.update({
            "track_id": self.next_id,
            "age": 1,
            "lost_count": 0,
            "is_lost": False,
        })
        self.next_id += 1
        return tr

    @staticmethod
    def _smooth_bbox(prev, cur, alpha):
        return tuple(int(round((1 - alpha) * p + alpha * c)) for p, c in zip(prev, cur))
