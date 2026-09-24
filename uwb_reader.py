"""
uwb_reader.py — UWB 거리 GT (선두 태그 ↔ 후미 앵커)

논문용 독립 계측이다. **제어에는 쓰지 않는다** — 추정기가 맞는지 재는 자(ground truth)가 추정기 안에 들어가면 자 노릇을
못 한다. main 은 값을 로그(`uwb.*`)에만 남기고, 분석은 analysis/nees_nis.py · analysis/sine_gain.py 가 한다.

구성 (docs/EXPERIMENT_PROTOCOL.md 2절):
  - 후미(팔로워) Jetson 에 UWB 모듈(예: Makerfabs ESP32 UWB DW3000, TWR initiator) 을 USB 로 붙이고 거리를 한 줄씩 찍게 한다
    → kind="serial". 선두에는 responder 만 있으면 되고 ESP-NOW 링크와 무관하다 (권장: GT 가 링크에 독립).
  - 또는 선두 ESP32 가 거리를 재서 텔레메트리 JSON 에 "uwb_range" 로 실어 보낸다 → kind="leader_packet".

기하: 모듈 안테나는 카메라 원점도, 선두의 시각적 중심(bbox 중심)도 아니다. 후미 안테나 오프셋 a(FRU, 카메라 원점 기준)와
선두 태그 오프셋 t(선두 시각 중심 기준)를 두고, EKF 가 주는 시선 단위벡터 u 로 |c| 를 푼다:
  r² = |ρ u + d|²,  d = t − a  →  ρ = −uᵀd + sqrt((uᵀd)² − |d|² + r²)   (uwb_range_to_center)
오프셋이 시선에 직교하면 보정은 2차(3 m 에서 0.1 m 오프셋 → 1.7 mm)라 실측 오프셋만 적어 두면 충분하다.
"""

import json
import math
import re
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class UwbRange:
    range_m: float
    rx_time: float          # time.monotonic() 수신 시각 (main 의 시계와 동일)
    seq: int = -1
    quality: Optional[float] = None
    raw: str = ""


_UNIT_SCALE = {"m": 1.0, "cm": 0.01, "mm": 0.001}
_KEYVAL_RE = re.compile(r"(uwb_range|range|distance|dist|d)\s*[:=]\s*([-+]?\d+(?:\.\d+)?)\s*(m|cm|mm)?\b", re.I)
_PLAIN_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)\s*(m|cm|mm)?\s*$", re.I)
_JSON_RANGE_KEYS = ("uwb_range", "range", "distance", "dist", "d", "range_m", "uwb")
_JSON_SEQ_KEYS = ("seq", "n", "count")
_JSON_Q_KEYS = ("quality", "q", "rssi", "fp_power")


def _reject_constant(name):
    raise ValueError(name)


def parse_uwb_line(text, unit="m", now=None):
    """한 줄 → UwbRange | None. 받는 형식:
      JSON  {"range": 3.12, "seq": 17, "q": -80}      (키 별칭: uwb_range/range/distance/dist/d/range_m/uwb)
      숫자  "3.12"  "312 cm"                         (단위 접미사가 있으면 그것을, 없으면 unit 인자를 쓴다)
      키값  "Range: 3.12 m"  "distance=312"           (Makerfabs/Decawave 예제 출력)
    비유한·0 이하는 None. 단위 인자는 m / cm / mm."""
    now = time.monotonic() if now is None else float(now)
    s = (text or "").strip()
    if not s:
        return None
    scale = _UNIT_SCALE.get(str(unit).lower(), 1.0)
    try:
        if s.startswith("{"):
            d = json.loads(s, parse_constant=_reject_constant)
            val = next((d[k] for k in _JSON_RANGE_KEYS if k in d), None)
            if val is None:
                return None
            r = float(val) * scale
            seq = int(next((d[k] for k in _JSON_SEQ_KEYS if k in d), -1))
            q = next((d[k] for k in _JSON_Q_KEYS if k in d), None)
            q = None if q is None else float(q)
        else:
            m = _PLAIN_RE.match(s) or _KEYVAL_RE.search(s)
            if m is None:
                return None
            num, suffix = (m.group(1), m.group(2)) if m.re is _PLAIN_RE else (m.group(2), m.group(3))
            r = float(num) * (_UNIT_SCALE[suffix.lower()] if suffix else scale)
            seq, q = -1, None
    except Exception:
        return None
    if not math.isfinite(r) or r <= 0.0:
        return None
    return UwbRange(range_m=r, rx_time=now, seq=seq, quality=q, raw=s)


class UwbRangeReceiver:
    """후미 쪽 UWB 모듈 시리얼 수신기. LeaderTelemetryReceiver 와 같은 줄 단위 드레인."""

    _RX_BUF_LIMIT = 65536

    def __init__(self, port="/dev/ttyUSB1", baud=115200, unit="m", min_m=0.2, max_m=60.0, timeout=0.001):
        self.port, self.baud, self.unit = port, int(baud), str(unit)
        self.min_m, self.max_m = float(min_m), float(max_m)
        self.timeout = timeout
        self.ser = None
        self._rx_buf = b""
        self._last_err = None
        self.latest: Optional[UwbRange] = None
        self.n_ok = self.n_bad = self.n_out_of_range = 0

    def start(self):
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        print(f"[UWB] serial opened: {self.port} @ {self.baud} unit={self.unit}")

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

    def _warn_once(self, msg):
        if msg != self._last_err:
            print(f"[UWB] {msg}")
            self._last_err = msg

    def feed_line(self, text, now=None):
        """한 줄을 파싱해 latest 를 갱신 (시리얼 드레인과 테스트가 같이 쓴다). 채택되면 True."""
        m = parse_uwb_line(text, unit=self.unit, now=now)
        if m is None:
            self.n_bad += 1
            return False
        if not (self.min_m <= m.range_m <= self.max_m):
            self.n_out_of_range += 1
            return False
        self.latest = m
        self.n_ok += 1
        return True

    def read_latest(self) -> Optional[UwbRange]:
        if self.ser is None:
            return self.latest
        try:
            n = int(getattr(self.ser, "in_waiting", 0) or 0)
            chunk = self.ser.read(n if n > 0 else 1)
        except Exception as exc:
            self._warn_once(f"serial read error: {exc}")
            return self.latest
        if chunk:
            self._rx_buf += chunk
            while b"\n" in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split(b"\n", 1)
                self.feed_line(line.decode("utf-8", errors="ignore"))
            if len(self._rx_buf) > self._RX_BUF_LIMIT:
                self._rx_buf = self._rx_buf[-4096:]
        return self.latest


def uwb_range_to_center(range_m, los_unit_fru, anchor_offset_fru=(0.0, 0.0, 0.0), tag_offset_fru=(0.0, 0.0, 0.0)):
    """앵커↔태그 거리 → 카메라 원점↔선두 시각 중심 거리. los_unit_fru 는 EKF 상대위치의 단위벡터(후미 FRU).
    tag_offset_fru 는 선두 태그 오프셋을 **후미 FRU 로 이미 돌린** 값(리더 heading 을 알면 formation.rotate_fru_about_up 으로,
    모르면 시선과 나란하다고 보고 그대로). 판별식이 음수(기하 모순)면 1차 근사 r − uᵀd 를 돌려준다."""
    r = float(range_m)
    u = np.asarray(los_unit_fru, dtype=float)
    nu = float(np.linalg.norm(u))
    if not (math.isfinite(r) and nu > 1e-9):
        return r
    u = u / nu
    d = np.asarray(tag_offset_fru, dtype=float) - np.asarray(anchor_offset_fru, dtype=float)
    ud = float(u @ d)
    disc = ud * ud - float(d @ d) + r * r
    if disc < 0.0:
        return r - ud
    return -ud + math.sqrt(disc)


def uwb_block(meas: Optional[UwbRange], now, max_age_sec, rel_fru=None, anchor_offset_fru=(0, 0, 0), tag_offset_fru=(0, 0, 0),
              source="serial"):
    """main 의 로그 한 블록. 제어에는 쓰지 않는다."""
    if meas is None:
        return {"available": False, "source": source, "reason": "no_uwb"}
    age = float(now) - float(meas.rx_time)
    if age > float(max_age_sec):
        return {"available": False, "source": source, "reason": "stale_uwb", "age": age, "range_m": meas.range_m, "seq": meas.seq}
    out = {"available": True, "source": source, "reason": "ok", "age": age, "range_m": float(meas.range_m), "seq": int(meas.seq),
           "quality": meas.quality, "range_center_m": float(meas.range_m), "range_est_m": None, "residual_m": None}
    if rel_fru is not None:
        rel = np.asarray(rel_fru, dtype=float)[:3]
        n = float(np.linalg.norm(rel))
        if math.isfinite(n) and n > 1e-6:
            out["range_center_m"] = float(uwb_range_to_center(meas.range_m, rel / n, anchor_offset_fru, tag_offset_fru))
            out["range_est_m"] = n
            out["residual_m"] = n - out["range_center_m"]
    return out
