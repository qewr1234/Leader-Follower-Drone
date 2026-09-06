"""
tracker.py — 선두 드론 단일 타겟 트래커

가벼운 IoU 기반 tracker.
나중에 OC-SORT/ByteTrack으로 교체 가능한 인터페이스를 유지한다.
"""

import math
from typing import List, Dict, Optional

from utils_geometry import iou_xyxy, bbox_area, bbox_center


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
        # 회복 중에는 겹침을 요구할 수 없다 — 블랙아웃 동안 타겟이 움직이기 때문이다.
        # 다만 "아무 검출이나 수용"은 안 된다: 검출률이 낮으면 lost_count>0이 정상
        # 상태가 되어 IoU 게이트가 사실상 무력화되고, 화면 반대편의 다른 대상이
        # 트랙을 가져간다. 순간이동만 막는 근접 게이트를 둔다.
        accept = best_iou >= self.iou_threshold or (
            self.track["lost_count"] > 0
            and self._near_enough(prev_bbox, best_det["bbox"], self.track["lost_count"])
        )

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

    def _near_enough(self, prev_bbox, det_bbox, lost_count):
        """중심 이동량이 놓친 프레임 수에 비례한 한계 안인가.

        허용 반경 = 이전 bbox 대각선 x (0.75 + 0.75 x lost_count).
        한 프레임(30fps 기준 33ms) 놓쳤을 때 bbox 크기의 0.75배까지 허용하므로
        정상 기동은 통과하고, 화면 반대편으로의 점프는 막힌다.
        """
        px, py = bbox_center(prev_bbox)
        dx, dy = bbox_center(det_bbox)
        diag = math.hypot(prev_bbox[2] - prev_bbox[0], prev_bbox[3] - prev_bbox[1])
        max_move = diag * (0.75 + 0.75 * float(lost_count))
        return math.hypot(dx - px, dy - py) <= max_move

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
