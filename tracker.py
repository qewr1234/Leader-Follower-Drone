"""
tracker.py — 선두 드론 단일 타겟 트래커

가벼운 IoU 기반 tracker.
나중에 OC-SORT/ByteTrack으로 교체 가능한 인터페이스를 유지한다.
"""

import math
from typing import List, Dict, Optional

from config import CONFIG
from utils_geometry import iou_xyxy, bbox_area, bbox_center

RECOVER_GATE_MIN_PX = float(CONFIG["scheduler"].get("recover_gate_min_px", 80))
# 새 트랙을 시작할 최소 검출 신뢰도 (config detector.init_conf_thres). 기존 트랙의 매칭에는 적용하지 않는다.
INIT_MIN_CONF = float(CONFIG["detector"].get("init_conf_thres", CONFIG["detector"].get("conf_thres", 0.25)))


class LeaderTracker:
    def __init__(self, iou_threshold=0.20, max_lost=12, recover_gate_frames=3):
        self.iou_threshold = iou_threshold
        self.max_lost = max_lost
        # 근접 게이트 반경이 lost_count에 비례해 커지는 것을 이 프레임 수에서 멈춘다.
        # 상한이 없으면 lost_count 8~9에서 반경이 화면 전체를 덮어 게이트가 사라진다.
        self.recover_gate_frames = int(recover_gate_frames)
        self.track = None
        self.next_id = 1
        self.gate_reject_streak = 0   # 추정기 3-D 게이트가 이 트랙의 거리 측정을 연속으로 거부한 횟수 (main 이 되먹임)

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

    def update(self, detections: List[Dict], reacquire_hint=None) -> Optional[Dict]:
        """detections: 검출 목록. reacquire_hint: 트랙이 없을 때만 쓰는 ((u, v), 반경 px) — 추정기가 예측하는 리더 화소 위치.
        힌트가 있으면 그 반경 안의 검출 중 예측점에 가장 가까운 것으로만 새 트랙을 만들고, 안에 아무것도 없으면 트랙을 만들지
        않는다(화면 어딘가의 더 큰 물체로 신원이 넘어가는 것을 막는다). 힌트가 없으면(추정기 미초기화·코스팅 만료) 예전처럼
        면적×신뢰도 최대 검출로 시작한다."""
        if self.track is None:
            # 새 트랙은 init_conf_thres 이상의 검출로만 시작한다 — YOLO 임계(0.25) 바로 위의 흔들리는 검출 한 장이 추정기 초기화까지
            # 끌고 가지 않게(이전 감사 #60). 매칭 중인 트랙은 낮은 신뢰도 프레임도 이어 간다.
            detections = [d for d in detections if float(d.get("conf", 0.0)) >= INIT_MIN_CONF]
            if not detections:
                return None
            if reacquire_hint is not None:
                det = self._choose_near(detections, *reacquire_hint)
                if det is None:
                    return None
            else:
                det = self._choose_initial(detections)
            self.track = self._new_track(det)
            self.gate_reject_streak = 0
            return self.track.copy()

        best_det = self._match(detections)
        if best_det is None:
            # 미검출이든 전부 게이트 밖이든 같은 미스다. 게이트 밖 검출을 미스로 세지 않으면
            # lost_count가 쌓이며 반경이 커져 결국 다른 대상이 트랙을 가져간다.
            return self._miss()

        alpha = 0.65 if self.track["lost_count"] == 0 else 0.85
        self.track["bbox"] = self._smooth_bbox(self.track["bbox"], best_det["bbox"], alpha)
        self.track["conf"] = float(best_det["conf"])
        self.track["area"] = bbox_area(self.track["bbox"])
        self.track["age"] += 1
        self.track["lost_count"] = 0
        self.track["is_lost"] = False
        return self.track.copy()

    def _miss(self) -> Dict:
        """놓친 프레임 하나 반영. max_lost를 넘기면 트랙을 버리고 마지막 상태를 돌려준다."""
        self.track["lost_count"] += 1
        self.track["is_lost"] = True
        self.track["conf"] *= 0.90
        old = self.track.copy()
        if self.track["lost_count"] > self.max_lost:
            self.track = None
            self.gate_reject_streak = 0
        return old

    def _match(self, detections):
        """게이트를 통과한 검출 중 점수 최대. 없으면 None.

        게이트를 먼저 걸어야 한다: 점수 최대 하나에만 게이트를 걸면 lost_count>0에서
        리더의 IoU≈0이라 conf가 높은 다른 대상이 뽑혀 거부되고, 게이트 안의 진짜 리더는
        검토조차 되지 않아 max_lost 뒤 폐기된다.
        회복 중(lost_count>0)에는 겹침을 요구할 수 없지만 "아무 검출이나 수용"도 안 된다 —
        순간이동만 막는 근접 게이트를 둔다.
        """
        prev_bbox = self.track["bbox"]
        lost_count = self.track["lost_count"]
        best_det, best_score = None, -1.0
        for det in detections:
            iou = iou_xyxy(prev_bbox, det["bbox"])
            if iou < self.iou_threshold and not (
                lost_count > 0 and self._near_enough(prev_bbox, det["bbox"], lost_count)
            ):
                continue
            score = 0.75 * iou + 0.25 * det["conf"]
            if score > best_score:
                best_score, best_det = score, det
        return best_det

    def _near_enough(self, prev_bbox, det_bbox, lost_count):
        """중심 이동량이 놓친 프레임 수에 비례한 한계 안인가.

        허용 반경 = 이전 bbox 대각선 x (0.75 + 0.75 x min(lost_count, recover_gate_frames)).
        한 프레임(30fps 기준 33ms) 놓쳤을 때 bbox 크기의 0.75배까지 허용하므로
        정상 기동은 통과하고, 화면 반대편으로의 점프는 막힌다. 반경은
        recover_gate_frames(기본 3 → 대각선 3.0배)에서 더 커지지 않는다 — 그 뒤로는
        "오래 놓쳤으니 어디든 받아준다"가 아니라 max_lost에서 트랙을 버리고 새로 시작한다.
        """
        px, py = bbox_center(prev_bbox)
        dx, dy = bbox_center(det_bbox)
        diag = math.hypot(prev_bbox[2] - prev_bbox[0], prev_bbox[3] - prev_bbox[1])
        grow = min(float(lost_count), float(self.recover_gate_frames))
        # 절대 하한: 먼 소형 표적(8~10 m, 12~14 px)은 대각선 비례 반경(≤61 px)이 자세 과도 한 번(10° ≈ 68 px)보다 작아 재매칭이 불가능했다.
        max_move = max(diag * (0.75 + 0.75 * grow), RECOVER_GATE_MIN_PX)
        return math.hypot(dx - px, dy - py) <= max_move

    def note_gate_accept(self):
        """추정기가 이 트랙의 거리 측정을 받아들였다."""
        self.gate_reject_streak = 0

    def note_gate_reject(self, drop_after: int):
        """추정기가 거리 측정을 거부했다. 연속 drop_after 회면 트랙을 버린다 — 다음 프레임에 reacquire_hint 로 다시 잡는다.
        (예전에는 거부가 트래커에 돌아오지 않아, 거부된 물체를 IoU 로 계속 붙들고 진짜 리더는 후보조차 되지 않았다.)"""
        self.gate_reject_streak += 1
        if self.track is not None and self.gate_reject_streak >= int(drop_after):
            self.track = None
            self.gate_reject_streak = 0
            return True
        return False

    def _choose_initial(self, detections):
        return max(detections, key=lambda d: d.get("area", 0) * max(d.get("conf", 0.0), 0.01))

    @staticmethod
    def _choose_near(detections, center, radius_px):
        """center 에서 radius_px 안에 있는 검출 중 가장 가까운 것. 없으면 None."""
        cx, cy = center
        best, best_d = None, float("inf")
        for det in detections:
            dx, dy = bbox_center(det["bbox"])
            d = math.hypot(dx - cx, dy - cy)
            if d <= radius_px and d < best_d:
                best, best_d = det, d
        return best

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
