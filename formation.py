"""
formation.py — 선두 1 : 후미 N 편대 토대 (formation slots · 리더 상태 방송 스키마 · 상대 heading 추정)

지금 main 은 슬롯을 하나만 안다: "내 시선(LOS) 기준 리더 뒤 TARGET_DISTANCE_M". 후미가 여럿이면 전부 같은 점으로
수렴해 충돌한다. 이 모듈은 그 한 슬롯을 일반화한다 — 슬롯을 리더 기준 프레임(또는 NED)에서 정의하고, 후미는
"리더 위치 + 회전된 오프셋" 이라는 가상 점을 추종한다. 기존 동작은 LOS 슬롯 (-TARGET_DISTANCE_M, 0, 0) 과 같다
(명령이 비트 단위로 같다 — `python3 test_closed_loop.py --compare` 로 확인).

레퍼런스 구현과의 대응 (docs/MULTI_FOLLOWER_FOUNDATION.md 에 출처)
- ArduPilot AP_Follow: FOLL_SYSID(리더 식별), FOLL_OFS_TYPE 0=NED / 1=리더 heading 기준, FOLL_OFS_X/Y/Z, FOLL_DIST_MAX,
  FOLL_YAW_BEHAVE 1=리더를 바라봄(기본), FOLLOW_TARGET(#144) 을 GLOBAL_POSITION_INT 보다 우선, 마지막 수신 뒤
  속도·가속도로 외삽, FOLL_TIMEOUT 3 s.  → FRAME_NED / FRAME_LEADER, leader_id 필터, LeaderState.
- PX4 follow_me (FlightTaskAutoFollowTarget): FLW_TGT_DST / FLW_TGT_FA(리더 heading 기준 각도) / FLW_TGT_HT, 리더 heading 은
  속도 방향에서 추정하되 1.0 m/s 미만에서는 동결.  → RelativeHeadingEstimator 의 속도 소스 + min_speed 데드존.
- MAVLink FOLLOW_TARGET(#144): timestamp[ms], est_capabilities(bit0 pos, bit1 vel, bit2 acc, bit3 att+rates),
  lat/lon[degE7], alt[m], vel[3]/acc[3](NED, m/s), attitude_q[4], rates[3], position_cov[3], custom_state.
  → LeaderState.to_follow_target() / from_follow_target().

문헌 근거
- 후미 각자가 "앞 기체" 만 보고 상수 간격을 유지하는 체인(predecessor-following)은 이중적분 기체 + 선형 제어기로는
  스트링 안정을 만들 수 없다(Seiler·Pant·Hedrick, IEEE TAC 49(10), 2004). 선두 속도를 모든 후미에 방송하면
  (leader-predecessor 토폴로지) 해소된다(Swaroop·Hedrick 1996/1999; Zheng 등, IEEE T-ITS 17(1), 2016).
  → main 의 ff_source="broadcast" 와 analysis.stability_margins.chain_sim(topology="leader_broadcast").
- 편대 제어 분류(Oh·Park·Ahn, Automatica 53, 2015): 상대 위치를 공통 방위에서 쓰려면(displacement-based) 후미가 리더
  heading 을 알아야 한다. 비전만으로는 후미 자신의 프레임밖에 없으므로 방송(yaw) 또는 속도 방향에서 얻는다.

좌표 규약
- FRU = [front, right, up] (main 과 동일). 슬롯 오프셋은 "리더 → 슬롯" 벡터.
- FRAME_LOS:    후미 자신의 FRU. 기존 동작. 리더 heading 불필요.
- FRAME_LEADER: 리더 FRU. rel_heading(= 리더 heading − 후미 heading, 우회전 +) 으로 회전해 후미 FRU 로 옮긴다.
- FRAME_NED:    [north, east, down]. 후미 yaw 로 후미 FRU 로 옮긴다.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

FRAME_LOS = "los"
FRAME_LEADER = "leader"
FRAME_NED = "ned"
FRAMES = (FRAME_LOS, FRAME_LEADER, FRAME_NED)


def wrap_pi(a):
    return float(math.atan2(math.sin(float(a)), math.cos(float(a))))


# ============================================================
# 슬롯
# ============================================================

@dataclass(frozen=True)
class FormationSlot:
    """편대 안의 자리 하나. offset 은 리더 → 슬롯 벡터(frame 규약은 모듈 docstring)."""
    slot_id: str
    offset: Tuple[float, float, float]
    frame: str = FRAME_LEADER
    follower_id: Optional[str] = None

    def __post_init__(self):
        if self.frame not in FRAMES:
            raise ValueError(f"unknown slot frame {self.frame!r} (one of {FRAMES})")
        if len(self.offset) != 3:
            raise ValueError("slot offset must have 3 components")
        object.__setattr__(self, "offset", tuple(float(v) for v in self.offset))

    def distance(self) -> float:
        return float(math.sqrt(sum(v * v for v in self.offset)))


def los_slot(distance, slot_id="behind_los") -> FormationSlot:
    """기존 동작: 후미 자신의 시선 기준 리더 뒤 distance."""
    return FormationSlot(slot_id, (-float(distance), 0.0, 0.0), FRAME_LOS)


def rotate_fru_about_up(v_fru, dpsi):
    """dpsi(우회전 +) 만큼 돌아간 프레임에서 표현된 FRU 벡터를 이 프레임의 FRU 로.
    돌아간 프레임의 forward 는 이 프레임에서 방위 +dpsi 에 있다: (cos, sin, 0). right 는 (−sin, cos, 0)."""
    c, s = math.cos(float(dpsi)), math.sin(float(dpsi))
    f, r, u = (float(x) for x in np.asarray(v_fru, dtype=float)[:3])
    return np.array([c * f - s * r, s * f + c * r, u], dtype=float)


def ned_offset_to_fru(offset_ned, follower_yaw):
    """[north, east, down] → 후미 FRU. yaw 는 북 기준 우회전 + (MAVLink ATTITUDE)."""
    n, e, d = (float(x) for x in np.asarray(offset_ned, dtype=float)[:3])
    c, s = math.cos(float(follower_yaw)), math.sin(float(follower_yaw))
    return np.array([n * c + e * s, -n * s + e * c, -d], dtype=float)


def slot_offset_fru(slot: FormationSlot, rel_heading=None, follower_yaw=None):
    """리더 → 슬롯 오프셋을 후미 FRU 로. 필요한 heading 이 없으면 None (호출자가 LOS 로 강등)."""
    if slot.frame == FRAME_LOS:
        return np.array(slot.offset, dtype=float)
    if slot.frame == FRAME_LEADER:
        return None if rel_heading is None else rotate_fru_about_up(slot.offset, rel_heading)
    return None if follower_yaw is None else ned_offset_to_fru(slot.offset, follower_yaw)


def enforce_min_distance(offset_fru, min_distance):
    """오프셋 길이가 min_distance 보다 짧으면 방향을 유지한 채 늘린다. 0 벡터면 후방으로.
    비전 거리 없이 GPS 상대위치뿐일 때 이격을 넓히는 기존 규칙(TARGET_DISTANCE_GPS_ONLY_M)의 일반형."""
    off = np.asarray(offset_fru, dtype=float)
    md = float(min_distance)
    if md <= 0.0:
        return off
    n = float(np.linalg.norm(off))
    if n < 1e-9:
        return np.array([-md, 0.0, 0.0], dtype=float)
    if n >= md:
        return off
    return off / n * md          # 단위벡터 먼저 — 축 정렬 슬롯이면 정확히 (-md, 0, 0)


def slot_error_fru(rel_leader_fru, slot: FormationSlot, rel_heading=None, follower_yaw=None, min_distance=0.0):
    """후미 → 슬롯 점 벡터(FRU). 제어기는 이것을 0 으로 만든다. 반환 (error, degraded_to_los).

    heading 이 필요한 슬롯인데 모르면 같은 거리의 LOS 후방 슬롯으로 강등한다(편대 모양은 잃지만 추종은 유지) —
    degraded_to_los=True 로 알린다. LOS 기본 슬롯 (-D, 0, 0) 은 rel + (-D, 0, 0) 이라 기존 (front − D, right, up) 과
    IEEE 754 상 동일한 값이다.
    """
    off = slot_offset_fru(slot, rel_heading, follower_yaw)
    degraded = False
    if off is None:
        degraded = True
        off = np.array([-slot.distance(), 0.0, 0.0], dtype=float)
    off = enforce_min_distance(off, min_distance)
    return np.asarray(rel_leader_fru, dtype=float)[:3] + off, degraded


# ============================================================
# config
# ============================================================

def slots_from_config(cfg: Dict[str, Any]) -> List[FormationSlot]:
    out = []
    for fid, s in (cfg or {}).get("slots", {}).items():
        out.append(FormationSlot(str(s.get("slot_id", f"slot_{fid}")), tuple(s.get("offset", (0.0, 0.0, 0.0))),
                                 str(s.get("frame", FRAME_LEADER)), follower_id=str(fid)))
    return out


def slot_from_config(cfg: Dict[str, Any], follower_id) -> Optional[FormationSlot]:
    """내 follower_id 의 슬롯. 없으면 None — main 은 매 프레임 los_slot(TARGET_DISTANCE_M) 을 쓴다(기존 동작)."""
    for s in slots_from_config(cfg):
        if s.follower_id == str(follower_id):
            return s
    return None


def validate_formation(slots: Sequence[FormationSlot], min_separation_m=2.0, depth_min_m=0.3, depth_max_m=10.0,
                       depth_reserve_m=3.0) -> List[str]:
    """정적 검사 (비행 전). 동적 충돌 회피는 아니다.
    - slot_id / follower_id 중복
    - 리더 기준 거리가 깊이창 안에 여유(depth_reserve_m — C4 의 '목표 + v/Kp' 여유)를 두고 드는가
    - 같은 프레임(leader / ned) 슬롯끼리 최소 이격. LOS 슬롯은 후미 위치에 따라 점이 달라 비교 불가 → 2개 이상이면 경고."""
    errors = []
    ids = [s.slot_id for s in slots]
    if len(set(ids)) != len(ids):
        errors.append(f"slot_id 중복: {sorted(i for i in ids if ids.count(i) > 1)}")
    fids = [s.follower_id for s in slots if s.follower_id is not None]
    if len(set(fids)) != len(fids):
        errors.append(f"follower_id 중복: {sorted(f for f in fids if fids.count(f) > 1)}")
    for s in slots:
        d = s.distance()
        if s.frame in (FRAME_LEADER, FRAME_LOS) and not (depth_min_m + 1.0 <= d <= depth_max_m - depth_reserve_m):
            errors.append(f"{s.slot_id}: 리더까지 {d:.2f} m — 깊이창 [{depth_min_m + 1.0:.1f}, {depth_max_m - depth_reserve_m:.1f}] 밖")
    for frame in (FRAME_LEADER, FRAME_NED):
        same = [s for s in slots if s.frame == frame]
        for i in range(len(same)):
            for j in range(i + 1, len(same)):
                sep = float(np.linalg.norm(np.subtract(same[i].offset, same[j].offset)))
                if sep < min_separation_m:
                    errors.append(f"{same[i].slot_id}↔{same[j].slot_id}: 이격 {sep:.2f} m < 최소 {min_separation_m:.1f} m")
    if sum(1 for s in slots if s.frame == FRAME_LOS) > 1:
        errors.append("LOS 슬롯이 2개 이상 — 각자 자기 시선 기준이라 같은 점으로 수렴할 수 있다")
    return errors


# ============================================================
# 상대 heading (리더 − 후미)
# ============================================================

class RelativeHeadingEstimator:
    """리더 heading − 후미 heading (rad, 우회전 +). FRAME_LEADER 슬롯을 후미 FRU 로 옮길 때 쓴다.

    소스 우선순위: 방송 yaw(ESP32 / FOLLOW_TARGET attitude_q) > 리더 속도 방향(수평 속도 ≥ min_speed) >
    최근값 유지(hold_sec) > None. PX4 follow_me 는 속도 방향만 쓰고 1.0 m/s 미만에서 동결한다 — 이 과제의 리더 속도
    (0.3 m/s 급)에서는 속도 소스가 거의 안 열리므로 방송 yaw 가 사실상 필수다."""

    def __init__(self, min_speed_mps=0.5, hold_sec=2.0):
        self.min_speed = float(min_speed_mps)
        self.hold_sec = float(hold_sec)
        self.reset()

    def reset(self):
        self.last = None
        self.last_t = None
        self.source = "none"

    def update(self, now, leader_yaw_ned=None, follower_yaw_ned=None, leader_vel_fru=None):
        """반환 (rel_heading | None, source). source ∈ broadcast / velocity / hold / none."""
        dpsi, src = None, None
        if leader_yaw_ned is not None and follower_yaw_ned is not None:
            dpsi, src = wrap_pi(float(leader_yaw_ned) - float(follower_yaw_ned)), "broadcast"
        elif leader_vel_fru is not None:
            v = np.asarray(leader_vel_fru, dtype=float)
            if v.size >= 2 and math.hypot(float(v[0]), float(v[1])) >= self.min_speed:
                dpsi, src = math.atan2(float(v[1]), float(v[0])), "velocity"
        if dpsi is not None and not math.isfinite(dpsi):
            dpsi = None          # 방송 yaw 가 NaN/inf 면 없는 것으로 (hold 로도 남기지 않는다)
        if dpsi is not None:
            self.last, self.last_t, self.source = dpsi, float(now), src
            return dpsi, src
        if self.last is not None and self.last_t is not None and float(now) - self.last_t <= self.hold_sec:
            self.source = "hold"
            return self.last, "hold"
        self.source = "none"
        return None, "none"


# ============================================================
# 리더 상태 방송 스키마 (FOLLOW_TARGET 호환)
# ============================================================

def _field(msg, name, default=None):
    if isinstance(msg, dict):
        return msg.get(name, default)
    return getattr(msg, name, default)


def _yaw_from_q(q):
    w, x, y, z = (float(v) for v in q[:4])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class LeaderState:
    """선두가 모든 후미에 방송하는 상태 하나. MAVLink FOLLOW_TARGET(#144) 과 필드가 1:1 이라 ESP-NOW JSON 이든
    MAVLink 든 같은 객체가 되고, 후미 N 대가 같은 방송을 받는다(leader-predecessor 토폴로지의 입력)."""
    timestamp: float                      # 송신측 시각 [s]
    lat: float
    lon: float
    alt: float
    alt_frame: str = "AMSL"
    vel_enu: Optional[np.ndarray] = None  # [east, north, up] m/s
    acc_enu: Optional[np.ndarray] = None
    yaw: Optional[float] = None           # rad, 북 기준 우회전 +
    leader_id: str = ""
    rx_time: float = 0.0
    seq: int = -1

    EST_POS, EST_VEL, EST_ACC, EST_ATT = 1, 2, 4, 8

    def est_capabilities(self) -> int:
        bits = self.EST_POS
        if self.vel_enu is not None:
            bits |= self.EST_VEL
        if self.acc_enu is not None:
            bits |= self.EST_ACC
        if self.yaw is not None:
            bits |= self.EST_ATT
        return bits

    @classmethod
    def from_leader_packet(cls, pkt, leader_velocity_frame="ENU"):
        """leader_telemetry.LeaderPacket → LeaderState. 속도 프레임은 main.LEADER_VELOCITY_FRAME 과 같은 규약."""
        if leader_velocity_frame.upper() == "NED":
            vel = np.array([pkt.vy, pkt.vx, -pkt.vz], dtype=float)
        else:
            vel = np.array([pkt.vx, pkt.vy, pkt.vz], dtype=float)
        acc = getattr(pkt, "acc", None)
        if acc is not None:
            acc = np.asarray(acc, dtype=float)
            if leader_velocity_frame.upper() == "NED":
                acc = np.array([acc[1], acc[0], -acc[2]], dtype=float)
        return cls(timestamp=float(pkt.timestamp), lat=float(pkt.lat), lon=float(pkt.lon), alt=float(pkt.alt),
                   alt_frame=str(getattr(pkt, "alt_frame", "AMSL")), vel_enu=vel, acc_enu=acc,
                   yaw=(None if getattr(pkt, "yaw", None) is None else float(pkt.yaw)),
                   leader_id=str(getattr(pkt, "leader_id", "") or ""), rx_time=float(getattr(pkt, "rx_time", 0.0)),
                   seq=int(getattr(pkt, "seq", -1)))

    def to_follow_target(self) -> Dict[str, Any]:
        """FOLLOW_TARGET 필드 dict (pymavlink follow_target_send 인자 순서와 같은 이름). vel/acc 는 NED."""
        def ned(v):
            return [0.0, 0.0, 0.0] if v is None else [float(v[1]), float(v[0]), -float(v[2])]
        q = [1.0, 0.0, 0.0, 0.0] if self.yaw is None else [math.cos(self.yaw / 2.0), 0.0, 0.0, math.sin(self.yaw / 2.0)]
        return {
            "timestamp": int(round(self.timestamp * 1000.0)),
            "est_capabilities": self.est_capabilities(),
            "lat": int(round(self.lat * 1e7)), "lon": int(round(self.lon * 1e7)), "alt": float(self.alt),
            "vel": ned(self.vel_enu), "acc": ned(self.acc_enu), "attitude_q": q,
            "rates": [0.0, 0.0, 0.0], "position_cov": [0.0, 0.0, 0.0], "custom_state": 0,
        }

    @classmethod
    def from_follow_target(cls, msg, leader_id="", now=None, alt_frame="AMSL"):
        """MAVLink FOLLOW_TARGET 메시지(또는 같은 키의 dict) → LeaderState. est_capabilities 비트가 없는 필드는 None."""
        caps = int(_field(msg, "est_capabilities", 0) or 0)
        vel = _field(msg, "vel")
        acc = _field(msg, "acc")
        q = _field(msg, "attitude_q")

        def enu(v):
            return None if v is None else np.array([float(v[1]), float(v[0]), -float(v[2])], dtype=float)
        return cls(timestamp=float(_field(msg, "timestamp", 0)) / 1000.0,
                   lat=float(_field(msg, "lat", 0)) * 1e-7, lon=float(_field(msg, "lon", 0)) * 1e-7,
                   alt=float(_field(msg, "alt", 0.0)), alt_frame=alt_frame,
                   vel_enu=enu(vel) if (caps & cls.EST_VEL) else None,
                   acc_enu=enu(acc) if (caps & cls.EST_ACC) else None,
                   yaw=(_yaw_from_q(q) if (q is not None and (caps & cls.EST_ATT)) else None),
                   leader_id=str(leader_id or ""), rx_time=(0.0 if now is None else float(now)))
