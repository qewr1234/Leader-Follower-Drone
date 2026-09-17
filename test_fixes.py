#!/usr/bin/env python3
"""C1~C6 + H2 회귀 테스트 — 단위 검사 86개.

하드웨어도 FC도 없이 순수 로직만 검증한다. cv2 / pymavlink / pyrealsense2 등은
sys.modules에 최소 스텁을 넣어 main.py를 import 가능하게 만든다.

    python3 test_fixes.py      # 전부 통과하면 exit 0
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  — {detail}" if detail else ""))
    if not ok:
        failures.append(name)


# ---------------------------------------------------------------- 스텁
def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _FakeMavlinkConsts:
    """MAVLink 표준 상수. 값은 규격에 고정되어 있으므로 하드코딩해도 안전하다."""
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 2
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 4
    POSITION_TARGET_TYPEMASK_VX_IGNORE = 8
    POSITION_TARGET_TYPEMASK_VY_IGNORE = 16
    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 64
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 128
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048
    MAV_FRAME_BODY_NED = 8
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_TYPE_GCS = 6
    MAV_CMD_NAV_LAND = 21
    MAV_DATA_STREAM_ALL = 0
    MAV_DATA_STREAM_EXTENDED_STATUS = 2
    MAV_DATA_STREAM_POSITION = 6
    MAV_DATA_STREAM_EXTRA1 = 10


for _name in ("cv2", "pyrealsense2", "serial"):
    _stub(_name)
_stub("ultralytics", YOLO=object)
_stub("pymavlink")
_stub("pymavlink.mavutil", mavlink=_FakeMavlinkConsts, mavutil=None)
sys.modules["pymavlink"].mavutil = sys.modules["pymavlink.mavutil"]
sys.modules["pymavlink.mavutil"].mode_string_v10 = lambda msg: msg.mode_name

from config import CONFIG                                    # noqa: E402
from mission_manager import (                                # noqa: E402
    MissionManager, S_WAIT_LEADER, S_FOLLOW, S_LEADER_HOVER, S_FAILSAFE_LAND,
)
from reliability import ReliabilityEstimator                 # noqa: E402

MOVING = dict(rel_est=[3.0, 0.0, 0.0], leader_alt=50.0, pos_cov_trace=1.0)


# ---------------------------------------------------------------- C1
m = MissionManager()
state, policy = m.update(now=100.0, leader_visible=False)
check("C1: 부팅 직후(리더 미획득) LAND 안 함",
      policy["land"] is False and state == S_WAIT_LEADER, f"state={state}")

m2 = MissionManager()
m2.update(now=100.0, leader_visible=True, rel_vel_est=[0.0, 0.0, 0.0], **MOVING)
state, policy = m2.update(now=105.0, leader_visible=False, **MOVING)
check("C1: 놓친 직후에는 LOST_HOLD (아직 착륙 아님)",
      policy["land"] is False, f"state={state}")
state, policy = m2.update(now=100.0 + m2.lost_hold_sec + 1.0,
                          leader_visible=False, **MOVING)
check("C1: lost_hold_sec 경과 후 FAILSAFE_LAND",
      policy["land"] is True and state == S_FAILSAFE_LAND, f"state={state}")

# ---------------------------------------------------------------- C3
m3 = MissionManager()
landed = False
for i in range(200):  # 착륙 조건(하강 + 정지)을 confirm_sec 넘게 유지
    _, p = m3.update(now=100.0 + i * 0.1, leader_visible=True,
                     rel_est=[3.0, 0.0, 0.0], rel_vel_est=[0.0, 0.0, -0.5],
                     leader_alt=None, pos_cov_trace=1.0)
    landed |= p["land"]
check("C3: 절대고도 없으면 공중 착륙판정 안 남", landed is False)

m4 = MissionManager()
landed = False
for i in range(200):
    _, p = m4.update(now=100.0 + i * 0.1, leader_visible=True,
                     rel_est=[3.0, 0.0, 0.0], rel_vel_est=[0.0, 0.0, -0.5],
                     leader_alt=0.2, pos_cov_trace=1.0)
    landed |= p["land"]
check("C3: 진짜 지면 근처(0.2m)에서는 착륙판정 남", landed is True)

# ---------------------------------------------------------------- H2
m5 = MissionManager()
t = 100.0
for i in range(30):  # 리더 이동 -> FOLLOW 진입
    st, _ = m5.update(now=t, leader_visible=True, rel_vel_est=[0.5, 0.0, 0.0], **MOVING)
    t += 0.1
check("H2: 이동하는 리더에서 FOLLOW 진입", st == S_FOLLOW, f"state={st}")

hover_frames, allow = 0, True
for i in range(50):  # 리더 정지 -> LEADER_HOVER 가 지속되어야 함
    st, p = m5.update(now=t, leader_visible=True, rel_vel_est=[0.0, 0.0, 0.0], **MOVING)
    t += 0.1
    if st == S_LEADER_HOVER:
        hover_frames += 1
        allow &= p["allow_follow"]
check("H2: LEADER_HOVER가 1프레임 넘게 지속", hover_frames >= 49, f"{hover_frames}프레임")
check("H2: LEADER_HOVER에서 정위치 유지 허용", allow is True)

# 히스테리시스: 0.18~0.25 구간에서는 상태가 바뀌지 않아야 한다
st_before = m5.state
st, _ = m5.update(now=t, leader_visible=True, rel_vel_est=[0.20, 0.0, 0.0], **MOVING)
check("H2: 히스테리시스 밴드(0.18~0.25)에서 상태 유지", st == st_before, f"{st_before}->{st}")
t += 0.1
st, _ = m5.update(now=t, leader_visible=True, rel_vel_est=[0.30, 0.0, 0.0], **MOVING)
check("H2: 0.25 초과에서 FOLLOW 복귀", st == S_FOLLOW, f"state={st}")

# ---------------------------------------------------------------- C4
margin = CONFIG["camera"]["depth_max_m"] - 3.0  # main.TARGET_DISTANCE_M
check("C4: 깊이 여유 >= 5m", margin >= 5.0, f"{margin:.1f}m")

rel = ReliabilityEstimator()
max_mad = CONFIG["measurement"]["max_depth_mad"]
bad = dict(depth_m=5.0, depth_valid_ratio=0.9, depth_mad=max_mad * 2)
good = dict(depth_m=5.0, depth_valid_ratio=0.9, depth_mad=max_mad * 0.1)
check("C4: MAD 한계 초과 측정은 신뢰도 0", rel.depth_reliability(bad) == 0.0,
      f"r={rel.depth_reliability(bad)}")
check("C4: 정상 측정은 신뢰도 > 0", rel.depth_reliability(good) > 0.0,
      f"r={rel.depth_reliability(good):.2f}")

# ---------------------------------------------------------------- C5
import main  # noqa: E402


class _CaptureMaster:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.sent = []
        self.mav = self
        self.mode_calls = []

    def set_position_target_local_ned_send(self, *a):
        self.sent.append(a)

    def mode_mapping(self):
        return {"LAND": (29, 4, 6)}          # PX4 형태(3-튜플)

    def set_mode(self, name):
        self.mode_calls.append(name)


BASE_MASK = 1 + 2 + 4 + 64 + 128 + 256 + 1024   # 1479

mst = _CaptureMaster()
main.send_body_velocity(mst, 0.5, 0.0, 0.0, yaw_rate=0.0)
mask = mst.sent[-1][4]
check("C5: yaw_rate=0에서도 YAW_RATE_IGNORE 안 세움", mask == BASE_MASK,
      f"mask={mask} (기대 {BASE_MASK})")

main.send_hold(mst)
check("C5: send_hold도 동일 마스크", mst.sent[-1][4] == BASE_MASK, f"mask={mst.sent[-1][4]}")
check("C5: 프레임은 BODY_NED(8)", mst.sent[-1][3] == 8)

# ---------------------------------------------------------------- C6
mst2 = _CaptureMaster()
ok = main.set_mode(mst2, "LAND")   # 3-튜플이어도 예외 없이 처리되어야 함
check("C6: PX4 3-튜플 mode_mapping에서 예외 없음", ok is True and mst2.mode_calls == ["LAND"],
      f"ok={ok} calls={mst2.mode_calls}")


class _RaisingMaster(_CaptureMaster):
    def set_mode(self, name):
        raise ValueError("simulated PX4 failure")


check("C6: set_mode 실패해도 예외 전파 안 함(fallback 도달 가능)",
      main.set_mode(_RaisingMaster(), "LAND") is False)

# ------------------------------------------------- 거리 관측 coast (소실 판정)
from imm_ekf import ImmEkf, RANGE_COAST_MAX_SEC  # noqa: E402

ek = ImmEkf()
ek.init([0.0, 0.0, 5.0])
check("rcoast: 초기화 직후 거리 확보", ek.has_range_fix())

# bearing-only 업데이트만 반복 → 거리는 관측되지 않는다
import numpy as np3  # noqa: E402
Rb = np3.diag([0.03 ** 2, 0.03 ** 2])
# coast_time을 먼저 0이 아니게 만들어 둔다. init 직후에는 0이라 "되돌린다"를 검사할 수 없다
# (예전 검사는 이 단계가 없어 update_bearing2d가 아무 일도 안 해도 통과했다).
ek.on_lost(0.5)
coast_raised = ek.coast_time
for _ in range(60):                     # 30fps 2초
    ek.predict(1 / 30)
    ek.update_bearing2d([0.0, 0.0], Rb)
check("rcoast: bearing-only는 coast_time을 되돌린다 (0.5 → 0)",
      coast_raised == 0.5 and ek.coast_time == 0.0 and ek.is_reliable(),
      f"before={coast_raised} after={ek.coast_time}")
for _ in range(30):                     # 다시 1초 → 총 3초 > 2.0
    ek.predict(1 / 30)
    ek.update_bearing2d([0.0, 0.0], Rb)
check("rcoast: bearing-only만으로는 거리 확보로 치지 않음",
      not ek.has_range_fix(),
      f"rcoast={ek.range_coast_time:.2f}s > {RANGE_COAST_MAX_SEC}")
check("rcoast: 그래도 is_reliable()은 참 — 둘이 다른 질문임을 확인",
      ek.is_reliable())

ek.update_position3d([0.0, 0.0, 5.0])   # RGB-D 측정 하나로 회복
check("rcoast: 거리 측정 들어오면 즉시 회복", ek.has_range_fix())

# 총 착륙 지연 = range_coast_max_sec + lost_hold_sec = 10초
total = CONFIG["imm"]["range_coast_max_sec"] + MissionManager().lost_hold_sec
check("소실 후 착륙까지 총 10초", abs(total - 10.0) < 1e-9, f"{total}s")

# ---------------------------------------------------------------- 트래커 신원 게이트
from tracker import LeaderTracker  # noqa: E402

tr = LeaderTracker()
tr.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
tr.update([])                                   # 1프레임 미검출 -> lost_count = 1
far = [{"bbox": [10.0, 10.0, 50.0, 50.0], "conf": 0.30, "cls_name": "person"}]
t = tr.update(far)
check("tracker: 1프레임 놓친 뒤 화면 반대편 검출은 거부",
      t["lost_count"] >= 2 and t["is_lost"], f"lost={t['lost_count']}")

tr2 = LeaderTracker()
tr2.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
tr2.update([])
near = [{"bbox": [312.0, 232.0, 352.0, 272.0], "conf": 0.5, "cls_name": "person"}]
t = tr2.update(near)
check("tracker: 근처 검출은 회복 허용", t["lost_count"] == 0 and not t["is_lost"])

# 리더는 사라지고 다른 대상만 계속 보이는 경우. 수정 전에는 게이트 반경이 lost_count에
# 비례해 무한히 커져 9프레임(0.3초)째에 반대편 검출이 트랙을 가져갔고, 거부 분기에
# max_lost가 없어 트랙이 죽지도 않았다.
tr3 = LeaderTracker()
tr3.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
stolen_at = None
dropped_at = None
for i in range(1, tr3.max_lost + 3):
    t = tr3.update(far)
    if tr3.track is None:
        dropped_at = i
        break
    if not t["is_lost"]:
        stolen_at = i
        break
check("tracker: 반대편 검출이 계속 있어도 기존 트랙을 넘겨주지 않음",
      stolen_at is None, f"{stolen_at}프레임째 탈취")
check("tracker: 게이트 밖 검출만 계속되면 max_lost에서 트랙 폐기",
      dropped_at == tr3.max_lost + 1, f"dropped_at={dropped_at} (기대 {tr3.max_lost + 1})")
t = tr3.update(far)
check("tracker: 폐기 뒤에는 새 track_id로 명시적 재초기화",
      t is not None and t["track_id"] == 2 and t["age"] == 1 and not t["is_lost"],
      f"track_id={t and t.get('track_id')} age={t and t.get('age')}")

# 상한이 정상 회복은 막지 않아야 한다: 3프레임 놓친 뒤 대각선 2배 거리(≈113px)는 통과
tr4 = LeaderTracker()
tr4.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
for _ in range(3):
    tr4.update([])
t = tr4.update([{"bbox": [380.0, 300.0, 420.0, 340.0], "conf": 0.6, "cls_name": "person"}])
check("tracker: 반경 상한 아래의 정상 회복은 여전히 허용", not t["is_lost"] and t["track_id"] == 1)

# 게이트-선행 회귀: 게이트 안의 리더(conf 0.5)와 반대편 오검출(conf 0.95)이 같이 보이는 경우.
# 수정 전 코드는 score = 0.75*IoU + 0.25*conf 최대인 검출 하나만 고른 뒤 그것에 게이트를
# 걸었다. 1프레임 놓친 뒤 리더는 45px 옮겨가 IoU가 0이라 두 검출의 score가 각각
# 0.125(리더) / 0.2375(오검출)로 오검출이 뽑히고, 오검출은 근접 게이트에서 거부되며,
# 게이트 안의 리더는 후보로 검토조차 안 된다 → 매 프레임 lost_count만 오르다 12프레임째
# (lost_count 13 > max_lost 12)에 트랙이 폐기된다. 게이트를 먼저 걸고 통과한 후보 중에서
# score 최대를 고르면 첫 프레임에 리더가 뽑혀 트랙이 유지된다.
tr6 = LeaderTracker()
tr6.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
tr6.update([])                                  # lost_count = 1 → 근접 게이트 활성
leader_bbox = [345.0, 265.0, 385.0, 305.0]      # 중심 이동 ≈64px < 허용 85px, IoU 0
mixed = [{"bbox": leader_bbox, "conf": 0.5, "cls_name": "person"},
         {"bbox": [10.0, 10.0, 50.0, 50.0], "conf": 0.95, "cls_name": "person"}]
for _ in range(12):
    t = tr6.update(mixed)
_cx, _cy = (t["bbox"][0] + t["bbox"][2]) / 2, (t["bbox"][1] + t["bbox"][3]) / 2
check("tracker: 게이트 안 리더 + 반대편 고신뢰 오검출이 함께 오면 리더로 트랙 유지",
      tr6.track is not None and not t["is_lost"] and t["track_id"] == 1
      and abs(_cx - 365.0) < 10 and abs(_cy - 285.0) < 10,
      f"is_lost={t['is_lost']} id={t['track_id']} center=({_cx:.0f},{_cy:.0f})")

# ---------------------------------------------------------------- FPS 독립 평활
import numpy as np2  # noqa: E402

zero, one = np2.zeros(4), np2.ones(4)
# 30fps에서 3프레임(0.1s) vs 24fps에서 2.4프레임 -> 같은 시간이면 같은 응답이어야 한다
v = zero.copy()
for _ in range(3):
    v = main.smooth_velocity_cmd(v, one, alpha=0.28, dt=1 / 30)
v30 = float(v[0])
v = zero.copy()
for _ in range(2):
    v = main.smooth_velocity_cmd(v, one, alpha=0.28, dt=1 / 24)
v24_2 = float(v[0])
v = main.smooth_velocity_cmd(zero, one, alpha=0.28, dt=1 / 24)
check("평활: 24fps 1스텝이 30fps 1스텝보다 빠르게 수렴 (dt 보정 동작)",
      float(v[0]) > 0.28, f"a_eff={float(v[0]):.3f} vs 0.280")
check("평활: 같은 경과시간(0.1s)이면 FPS가 달라도 응답 근사 일치",
      abs(v30 - v24_2) < 0.06, f"30fps={v30:.3f} 24fps={v24_2:.3f}")
check("평활: dt 미지정이면 기존 동작 유지",
      abs(float(main.smooth_velocity_cmd(zero, one, alpha=0.28)[0]) - 0.28) < 1e-9)

# ------------------------------------------------- detector_skipped 전파
# scheduler가 검출을 건너뛴 프레임은 마지막 bbox에 새 depth를 씌운 측정이다.
# reliability는 track의 detector_skipped를 보고 0.55를 곱하는데, measurement dict가
# 이 키를 복사하지 않아 main.py의 vision_reliability(rgbd_meas or ...) 경로에서는
# 페널티가 한 번도 적용되지 않았다.
from measurement import MeasurementBuilder  # noqa: E402

_intr = {"fx": 384.0, "fy": 384.0, "ppx": 320.0, "ppy": 240.0}
_mb = MeasurementBuilder(_intr, depth_scale=0.001)
_depth = np2.full((480, 640), 4000, dtype=np2.uint16)     # 4m 평면
_rel = ReliabilityEstimator()

tr5 = LeaderTracker()
for _ in range(6):
    tr5.update([{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}])
m_live = _mb.build_rgbd(tr5.update(
    [{"bbox": [300.0, 220.0, 340.0, 260.0], "conf": 0.9, "cls_name": "person"}]), _depth)
m_skip = _mb.build_rgbd(tr5.predict_only(), _depth)
check("skip: 건너뛴 프레임의 rgbd 측정에 detector_skipped 키 전파",
      m_skip.get("detector_skipped") is True and m_live.get("detector_skipped") is False)
r_live, r_skip = _rel.vision_reliability(m_live), _rel.vision_reliability(m_skip)
check("skip: 건너뛴 프레임 측정의 신뢰도가 0.55배로 깎임",
      r_live > 0 and abs(r_skip / r_live - 0.55) < 1e-6, f"live={r_live:.3f} skip={r_skip:.3f}")
b_skip = _mb.build_bearing(tr5.predict_only())
check("skip: bearing 측정에도 전파", b_skip.get("detector_skipped") is True)

# ------------------------------------------------- 재획득 시 추종 재개
# 잠깐 놓쳤다(LOST_HOLD) 다시 찾았을 때, 리더가 호버 중이면 상대속도가 0이라
# 출발 조건(0.25 m/s)이 영원히 안 만족돼 READY_HOVER에 갇혔다. 그 상태에선 yaw 제어도
# 안 돌아 리더가 천천히 시야를 벗어나면 소실 착륙으로 이어진다.
from mission_manager import S_LOST_HOLD, S_READY_HOVER  # noqa: E402

_MV = dict(rel_est=[3.0, 0.0, 0.0], leader_alt=50.0, pos_cov_trace=1.0)


def _follow_then_lose(m, t, lose_sec):
    for _ in range(30):
        st, _ = m.update(now=t, leader_visible=True, rel_vel_est=[0.5, 0, 0], **_MV); t += 0.1
    assert st == S_FOLLOW
    for _ in range(int(lose_sec * 10)):
        st, _ = m.update(now=t, leader_visible=False, **_MV); t += 0.1
    return st, t


m6 = MissionManager()
st, t = _follow_then_lose(m6, 100.0, 2.0)
check("재개: 2초 소실은 LOST_HOLD", st == S_LOST_HOLD, f"state={st}")
st, p = m6.update(now=t, leader_visible=True, rel_vel_est=[0.0, 0, 0], **_MV)
check("재개: 호버 중인 리더를 재획득하면 즉시 LEADER_HOVER(allow_follow)",
      st == S_LEADER_HOVER and p["allow_follow"] is True, f"state={st} allow={p['allow_follow']}")

m7 = MissionManager()
st, t = _follow_then_lose(m7, 100.0, 2.0)
st, p = m7.update(now=t, leader_visible=True, rel_vel_est=[0.5, 0, 0], **_MV)
check("재개: 이동 중인 리더를 재획득하면 출발 확인 없이 즉시 FOLLOW",
      st == S_FOLLOW and p["allow_follow"] is True, f"state={st}")

m8 = MissionManager()                      # 아직 한 번도 FOLLOW한 적 없음
m8.update(now=100.0, leader_visible=True, rel_vel_est=[0.0, 0, 0], **_MV)
for i in range(20):
    m8.update(now=100.1 + i * 0.1, leader_visible=False, **_MV)
st, p = m8.update(now=102.2, leader_visible=True, rel_vel_est=[0.0, 0, 0], **_MV)
check("재개: 추종한 적 없으면 기존대로 READY_HOVER (출발 확인 필요)",
      st == S_READY_HOVER and p["allow_follow"] is False, f"state={st}")

m9 = MissionManager()
st, t = _follow_then_lose(m9, 100.0, m9.lost_hold_sec + 1.0)
check("재개: 오래 소실은 FAILSAFE_LAND", st == S_FAILSAFE_LAND, f"state={st}")
st, p = m9.update(now=t, leader_visible=True, rel_vel_est=[0.0, 0, 0], **_MV)
check("재개: FAILSAFE_LAND 뒤의 재획득은 자동 재개하지 않음 (READY_HOVER)",
      st == S_READY_HOVER and p["allow_follow"] is False, f"state={st}")

m10 = MissionManager()
_follow_then_lose(m10, 100.0, 1.0)
m10.reset()                                 # GUIDED 인계 시 main이 호출
check("재개: reset()이 '추종한 적 있음' 기억까지 지움 (인계 후 출발 확인 재요구)",
      m10.has_followed is False and m10.state == S_WAIT_LEADER and m10.last_seen_t is None)

# 착륙 확인 타이머는 가림 중에 끊어야 한다. 남겨 두면 가려진 시간이 합산돼 재획득
# 첫 프레임에 (now - candidate_t)가 confirm_sec을 넘어 LAND가 나간다.
from mission_manager import S_LANDING_CANDIDATE, S_CONFIRMED_LANDING  # noqa: E402

_LANDING = dict(rel_est=[3.0, 0.0, 0.0], rel_vel_est=[0.0, 0.0, -0.5], leader_alt=0.3, pos_cov_trace=1.0)
m11 = MissionManager()
t = 100.0
for _ in range(10):                          # 착륙 후보 1.0초 (< confirm 1.8초)
    st, p = m11.update(now=t, leader_visible=True, **_LANDING); t += 0.1
check("타이머: 1.0초 착륙 후보는 아직 LANDING_CANDIDATE",
      st == S_LANDING_CANDIDATE and p["land"] is False, f"state={st}")
for _ in range(30):                          # 3초 소실 (< lost_hold 8초)
    st, p = m11.update(now=t, leader_visible=False, rel_est=[3.0, 0.0, 0.0], pos_cov_trace=1.0); t += 0.1
assert st == S_LOST_HOLD, st
t_reacq = t
st, p = m11.update(now=t, leader_visible=True, **_LANDING); t += 0.1
check("타이머: 3초 소실 뒤 재획득 첫 프레임에 land 안 나감 (가려진 시간 합산 금지)",
      st == S_LANDING_CANDIDATE and p["land"] is False
      and m11.landing_candidate_t == t_reacq,
      f"state={st} land={p['land']} candidate_t={m11.landing_candidate_t} (기대 {t_reacq})")
st, p = m11.update(now=t_reacq + m11.landing_confirm_sec - 0.05, leader_visible=True, **_LANDING)
before = (st, p["land"])                    # 새로 1.8초를 채우기 직전
st, p = m11.update(now=t_reacq + m11.landing_confirm_sec + 0.05, leader_visible=True, **_LANDING)
check("타이머: 재획득 시점부터 confirm_sec(1.8초)을 새로 채워야 LAND",
      before == (S_LANDING_CANDIDATE, False) and st == S_CONFIRMED_LANDING and p["land"] is True,
      f"before={before} after={st}")

# ------------------------------------------------- 현장 대비 (2026-09-07 감사)
# ESP32는 선택 사항이다. 플래그가 켜져 있어도 장치가 없으면 startup에서 죽지 않고
# 비전 단독으로 가야 한다. (예전에는 플래그 자체를 False로 강제했지만, 사용자가 켠 뒤
# 테스트만 깨져 있었다 — 검사할 것은 플래그 값이 아니라 "없어도 안 죽는다"다.)
_saved = (main.USE_LEADER_ESP32, main.LEADER_TELEMETRY_KIND)
main.USE_LEADER_ESP32 = True
main.LEADER_TELEMETRY_KIND = "serial"


class _NoDevice(Exception):
    pass


def _raise_no_device(*a, **k):
    raise _NoDevice("could not open port /dev/ttyUSB0")


sys.modules["serial"].Serial = _raise_no_device
try:
    rx = main.open_leader_receiver()
    check("ESP32: 장치가 없어도 startup crash 없음 (None 반환)", rx is None, f"rx={rx!r}")
except Exception as e:
    check("ESP32: 장치가 없어도 startup crash 없음 (None 반환)", False,
          f"{type(e).__name__}: {e}")

del sys.modules["serial"].Serial          # pyserial 자체가 없는 환경(AttributeError)
try:
    rx = main.open_leader_receiver()
    check("ESP32: pyserial이 없어도 startup crash 없음", rx is None, f"rx={rx!r}")
except Exception as e:
    check("ESP32: pyserial이 없어도 startup crash 없음", False, f"{type(e).__name__}: {e}")

main.USE_LEADER_ESP32 = False
check("ESP32: 플래그 off면 수신기를 열지 않음", main.open_leader_receiver() is None)
main.USE_LEADER_ESP32, main.LEADER_TELEMETRY_KIND = _saved

check("헤드리스: MARS_SHOW_WINDOW로 창을 끌 수 있음",
      "MARS_SHOW_WINDOW" in open("main.py", encoding="utf-8").read())

check("AGL 바닥 상수 존재", main.MIN_AGL_M > 0, f"MIN_AGL_M={main.MIN_AGL_M}")
check("카메라 연속 실패 한계 존재", main.CAM_FAIL_LIMIT > 0,
      f"CAM_FAIL_LIMIT={main.CAM_FAIL_LIMIT}")

# 검출 클래스 불일치 경고
import io as _io  # noqa: E402
import contextlib  # noqa: E402
from detector import YoloDetector  # noqa: E402


class _FakeModel:
    names = {0: "leader_drone"}


buf = _io.StringIO()
with contextlib.redirect_stdout(buf):
    YoloDetector(model=_FakeModel(), target_class_name="person")
check("검출: 모델에 없는 클래스면 시작 시 크게 경고",
      "target_class_name='person'" in buf.getvalue() and "없습니다" in buf.getvalue())

buf = _io.StringIO()
with contextlib.redirect_stdout(buf):
    YoloDetector(model=_FakeModel(), target_class_name="leader_drone")
check("검출: 클래스가 맞으면 확인 메시지", "확인됨" in buf.getvalue())

# ------------------------------------------------- HEARTBEAT 컴포넌트 필터
# 같은 시스템 ID의 다른 컴포넌트(짐벌·카메라·라우터)가 heartbeat를 내면 모드 문자열이
# 왕복해 main의 GUIDED 진입 에지가 매번 발동하고 미션이 계속 리셋됐다.
import mavlink_io  # noqa: E402


class _HB:
    def __init__(self, src_sys, src_comp, mode_name, mav_type=2, armed=True):
        self._sys, self._comp = src_sys, src_comp
        self.mode_name, self.type = mode_name, mav_type
        self.base_mode = 128 if armed else 0

    def get_type(self): return "HEARTBEAT"
    def get_srcSystem(self): return self._sys
    def get_srcComponent(self): return self._comp


class _SysStatus:
    def __init__(self, voltage): self.voltage_battery, self.battery_remaining = voltage, -1
    def get_type(self): return "SYS_STATUS"


class _Master:
    target_system, target_component = 1, 1

    def __init__(self, msgs): self._q = list(msgs) + [None]
    def recv_match(self, blocking=False): return self._q.pop(0)


mavlink_io.drain_messages(_Master([_HB(1, 1, "GUIDED")]))
check("HB: autopilot 컴포넌트의 heartbeat로 모드 갱신",
      mavlink_io.get_vehicle_state()["mode"]["name"] == "GUIDED")
mavlink_io.drain_messages(_Master([_HB(1, 154, "Mode(0)", mav_type=26)]))   # 짐벌
check("HB: 같은 시스템의 다른 컴포넌트 heartbeat는 무시",
      mavlink_io.get_vehicle_state()["mode"]["name"] == "GUIDED",
      f"mode={mavlink_io.get_vehicle_state()['mode']['name']}")
mavlink_io.drain_messages(_Master([_HB(255, 190, "LOITER", mav_type=6)]))     # GCS
check("HB: GCS heartbeat는 무시 (기존 동작 유지)",
      mavlink_io.get_vehicle_state()["mode"]["name"] == "GUIDED")

# ------------------------------------------------- 배터리 전압 sentinel
mavlink_io.last_battery_voltage = None
mavlink_io.drain_messages(_Master([_SysStatus(65535)]))
check("BAT: voltage_battery=65535(미보고)는 전압으로 안 씀",
      mavlink_io.last_battery_voltage is None, f"v={mavlink_io.last_battery_voltage}")
mavlink_io.drain_messages(_Master([_SysStatus(12600)]))
check("BAT: 정상 전압은 V로 환산", mavlink_io.last_battery_voltage == 12.6)

# ------------------------------------------------- connect_fc heartbeat 대기
# wait_heartbeat(timeout=10)은 timeout 시 None을 돌려준다. connect_fc는 포기하지 않고
# 10초마다 대기 메시지를 찍으며 heartbeat가 올 때까지 다시 기다려야 한다.


class _FakeFCMaster:
    target_system, target_component = 1, 7

    def __init__(self):
        self.mav = self
        self.hb_calls = []
        self.stream_calls = []

    def wait_heartbeat(self, timeout=None):
        self.hb_calls.append(timeout)
        return None if len(self.hb_calls) < 3 else self     # 처음 2번은 timeout

    def request_data_stream_send(self, *a):
        self.stream_calls.append(a)


_fake_fc = _FakeFCMaster()
_mavutil_stub = sys.modules["pymavlink.mavutil"]
_mavutil_stub.mavlink_connection = lambda port, baud=None: _fake_fc
buf = _io.StringIO()
try:
    with contextlib.redirect_stdout(buf):
        _master = mavlink_io.connect_fc()
    check("FC: heartbeat가 2번 timeout 돼도 예외 없이 master 반환 (3번째에 획득)",
          _master is _fake_fc and _fake_fc.hb_calls == [10, 10, 10],
          f"hb_calls={_fake_fc.hb_calls}")
except Exception as e:
    check("FC: heartbeat가 2번 timeout 돼도 예외 없이 master 반환 (3번째에 획득)", False,
          f"{type(e).__name__}: {e}")
finally:
    del _mavutil_stub.mavlink_connection
_out = buf.getvalue()
check("FC: 대기 메시지(10s, 20s) 출력 + 연결 후 data stream 4개 요청",
      "heartbeat 대기 중 (10s)" in _out and "heartbeat 대기 중 (20s)" in _out
      and "connected" in _out and len(_fake_fc.stream_calls) == 4
      and all(c[:2] == (1, 7) for c in _fake_fc.stream_calls),
      f"streams={len(_fake_fc.stream_calls)} out={_out.strip().splitlines()}")

# ------------------------------------------------- ESP32 serial 부분 라인
# readline()+1ms timeout은 전송 도중 읽으면 앞토막/뒤토막이 따로 잘려 패킷을 통째로 잃었다.
from leader_telemetry import LeaderTelemetryReceiver, parse_leader_json  # noqa: E402
from leader_telemetry import build_leader_measurement_from_packet  # noqa: E402


class _FakeSerial:
    def __init__(self, chunks): self._chunks = list(chunks)
    @property
    def in_waiting(self): return len(self._chunks[0]) if self._chunks else 0
    def read(self, n): return self._chunks.pop(0) if self._chunks else b""


_pkt = b'{"lat":35.83,"lon":128.75,"alt":50.0,"vx":0.1,"vy":0.2,"vz":0.0,"seq":%d}\n'
rx = LeaderTelemetryReceiver(kind="serial")
rx.ser = _FakeSerial([
    (_pkt % 1)[:30],                                  # 1번 패킷 앞토막
    (_pkt % 1)[30:] + (_pkt % 2) + (_pkt % 3)[:12],   # 뒤토막 + 2번 전체 + 3번 앞토막
])
check("serial: 잘린 앞토막만 왔을 때는 패킷 없음 (버림도 없음)",
      rx.read_latest() is None and rx._rx_buf == (_pkt % 1)[:30])
p2 = rx.read_latest()
check("serial: 뒤토막이 오면 1번을 복원하고, 같은 chunk의 2번까지 파싱해 최신(2)을 반환",
      p2 is not None and p2.seq == 2, f"seq={p2 and p2.seq}")
check("serial: 3번 앞토막은 버퍼에 남아 다음 읽기를 기다림",
      rx._rx_buf == (_pkt % 3)[:12])

# ------------------------------------------------- 리더 고도 기준계
_ok = parse_leader_json('{"lat":1,"lon":2,"alt_ellipsoid":62.0}')
check("alt: alt_ellipsoid 필드는 ELLIPSOID", _ok.alt_frame == "ELLIPSOID" and _ok.alt == 62.0)
check("alt: alt_msl 필드는 AMSL", parse_leader_json('{"lat":1,"lon":2,"alt_msl":37}').alt_frame == "AMSL")
check("alt: 그냥 alt는 수신기 기본값을 따름",
      parse_leader_json('{"lat":1,"lon":2,"alt":37}').alt_frame == "AMSL"
      and parse_leader_json('{"lat":1,"lon":2,"alt":62}', default_alt_frame="ellipsoid").alt_frame == "ELLIPSOID")

_FOLLOWER = {
    "global_position": {"lat": 358300000, "lon": 1287500000, "alt": 35000, "timestamp": 1.0},  # 35m AMSL
    "gps": {"lat": 358300000, "lon": 1287500000, "alt": 35000, "alt_ellipsoid": 60000,          # 60m 타원체고
            "timestamp": 1.0},
    "attitude": {"yaw": 0.0, "timestamp": 1.0},
}
_pk = lambda js: parse_leader_json(js)  # noqa: E731
m_amsl = build_leader_measurement_from_packet(_pk('{"lat":35.83,"lon":128.75,"alt_msl":37.0}'), _FOLLOWER, now=0.0)
m_ell = build_leader_measurement_from_packet(_pk('{"lat":35.83,"lon":128.75,"alt_ellipsoid":62.0}'), _FOLLOWER, now=0.0)
check("alt: AMSL 리더는 팔로워 GLOBAL_POSITION_INT.alt와 뺌 → up=+2.0",
      m_amsl["available"] and abs(m_amsl["rel_fru"][2] - 2.0) < 1e-6, f"up={m_amsl.get('rel_fru')}")
check("alt: 타원체고 리더는 팔로워 alt_ellipsoid와 뺌 → up=+2.0 (지오이드 25m 안 섞임)",
      m_ell["available"] and abs(m_ell["rel_fru"][2] - 2.0) < 1e-6, f"up={m_ell.get('rel_fru')}")
_no_ell = {k: dict(v) for k, v in _FOLLOWER.items()}
_no_ell["gps"].pop("alt_ellipsoid")
m_bad = build_leader_measurement_from_packet(_pk('{"lat":35.83,"lon":128.75,"alt_ellipsoid":62.0}'), _no_ell, now=0.0)
check("alt: 타원체고 리더인데 팔로워 타원체고가 없으면 측정 불가로 거부 (섞어 쓰지 않음)",
      m_bad["available"] is False and m_bad["reason"] == "no_follower_ellipsoid_alt", f"{m_bad}")

# MAVLink lat/lon은 deg*1e7, alt는 mm — 항상 그 배율로 나눈다. 크기로 단위를 추측하면
# 해발 1m 미만 이륙지에서 alt=800mm 가 800m 로 남는다.
from leader_telemetry import normalize_lat_lon_alt_from_mavlink  # noqa: E402

_lla = normalize_lat_lon_alt_from_mavlink({"lat": 358300000, "lon": 1287500000, "alt": 800})
check("lla: MAVLink 정수 (358300000, 1287500000, 800mm) → (35.83, 128.75, 0.8m)",
      _lla is not None and all(abs(a - b) < 1e-9 for a, b in zip(_lla, (35.83, 128.75, 0.8))),
      f"{_lla}")

# ------------------------------------------------- GPS 단독 거리 → 이격 확대
# has_range_fix()는 ESP32 GPS 상대위치로도 참이 된다. 비전이 죽어도 소실 판정은 안 나는 게
# 맞지만(링크가 살아 있으니), GPS 오차(m 단위)만으로 3m 이격 추종을 계속하면 안 된다.
ek2 = ImmEkf()
ek2.init([0.0, 0.0, 8.0], source="gps")
check("gps-only: GPS로 초기화하면 거리는 알지만 비전 거리는 없음",
      ek2.has_range_fix() and not ek2.has_vision_range_fix())
ek2.update_position3d([0.0, 0.0, 8.0], source="rgbd")
check("gps-only: RGB-D 측정이 들어오면 비전 거리 확보", ek2.has_vision_range_fix())
for _ in range(90):                                   # 3초간 GPS만
    ek2.predict(1 / 30)
    ek2.update_position3d([0.0, 0.0, 8.0], np3.diag([4.0, 4.0, 9.0]), source="gps")
check("gps-only: GPS만 3초면 비전 거리는 만료, 전체 거리는 유지 (소실 판정 안 남)",
      ek2.has_range_fix() and not ek2.has_vision_range_fix(),
      f"range={ek2.range_coast_time:.2f} vision={ek2.vision_range_coast_time:.2f}")
check("gps-only: 이격 거리 TARGET < GPS_ONLY < depth_max (비전이 다시 이어받을 수 있는 범위)",
      main.TARGET_DISTANCE_M < main.TARGET_DISTANCE_GPS_ONLY_M < CONFIG["camera"]["depth_max_m"],
      f"{main.TARGET_DISTANCE_M} < {main.TARGET_DISTANCE_GPS_ONLY_M} < {CONFIG['camera']['depth_max_m']}")
cmd = main.compute_velocity_cmd_from_estimate([8.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0,
                                              target_distance=main.TARGET_DISTANCE_GPS_ONLY_M)
check("gps-only: target_distance=8m이면 8m에서 전진 명령 0", abs(cmd[0]) < 1e-9, f"vx={cmd[0]:.3f}")
cmd = main.compute_velocity_cmd_from_estimate([8.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0)
check("gps-only: 기본 target은 여전히 3m (8m에서는 전진)", cmd[0] > 0.3, f"vx={cmd[0]:.3f}")

# ------------------------------------------------- 죽은 코드 제거 / 하네스 범위
import detector as _det  # noqa: E402
check("정리: controller.py(모터 테스트 프로토타입) 제거", not Path("controller.py").exists())
check("정리: mavlink_io 모터 테스트 계열 제거",
      not any(hasattr(mavlink_io, n) for n in ("motor_test_percent", "trigger_motors", "stop_all_motors")))
check("정리: detector.select_target 제거", not hasattr(_det, "select_target"))
_h = open("sitl/harness.py", encoding="utf-8").read()
check("하네스: --all이 depth_loss·handover까지 포함",
      '"depth_loss", "handover"] if ARGS.all' in _h)

# ------------------------------------------------- 불확실성 분기의 타이머 초기화
# pos_cov_trace > 8.0 이면 리더가 보여도 LOST_HOLD 로 간다. 이때도 착륙 후보 타이머를 끊어야
# 공분산이 회복된 첫 프레임에 (가려진 시간이 합산돼) CONFIRMED_LANDING 이 나지 않는다.
m12 = MissionManager()
t = 100.0
_LANDING = dict(rel_est=[3.0, 0.0, 0.0], rel_vel_est=[0.0, 0.0, -0.5], leader_alt=0.3)
for _ in range(10):                              # 1.0초 착륙 후보
    m12.update(now=t, leader_visible=True, pos_cov_trace=1.0, **_LANDING); t += 0.1
for _ in range(30):                              # 3초 공분산 폭주 → LOST_HOLD
    st, _ = m12.update(now=t, leader_visible=True, pos_cov_trace=20.0, **_LANDING); t += 0.1
check("타이머: 공분산 폭주 중 LOST_HOLD", st == S_LOST_HOLD, f"state={st}")
st, p = m12.update(now=t, leader_visible=True, pos_cov_trace=1.0, **_LANDING)
check("타이머: 공분산 회복 첫 프레임에 착륙 명령 없음 (타이머 새로 시작)",
      p["land"] is False and m12.landing_candidate_t == t, f"land={p['land']} cand_t={m12.landing_candidate_t} t={t}")

# ------------------------------------------------- ego-yaw 보정이 CT omega 로 새지 않음
# 타겟이 월드에서 직진(참 선회율 0)하고 팔로워가 일정 yaw rate 로 회전하면, compensate_ego_yaw 가
# 속도 벡터를 dpsi 만큼 돌린다. 직전 방향각(_prev_heading)을 같이 돌리지 않으면 d_heading 에
# dpsi 가 통째로 섞여 omega 가 팔로워 yaw rate 로 수렴했다(수정 전 0.2 rad/s → omega +0.202).
import math as _math  # noqa: E402


def _omega_with_ego_yaw(yaw_rate, v_w=1.0):
    ek = ImmEkf(); ek.init([0.0, 0.0, 5.0])
    dt = 1 / 30; Rm = np3.diag([0.15 ** 2, 0.15 ** 2, 0.25 ** 2]); psi = 0.0
    for k in range(1, 301):
        t = k * dt
        ek.predict(dt)
        psi += yaw_rate * dt
        ek.compensate_ego_yaw(yaw_rate * dt)
        r = _math.hypot(v_w * t, 5.0); phi_c = _math.atan2(5.0, v_w * t) + psi   # 카메라 각 = 월드 각 + ψ
        ek.update_position3d(np3.array([r * _math.cos(phi_c), 0.0, r * _math.sin(phi_c)]), Rm)
    return ek.filters[1]._omega


check("ego-yaw: 팔로워 yaw 0.2 rad/s 가 CT omega 로 새지 않음 (|omega| < 0.02)",
      abs(_omega_with_ego_yaw(0.2)) < 0.02, f"omega={_omega_with_ego_yaw(0.2):+.3f}")
check("ego-yaw: 반대 방향(-0.3 rad/s)도 동일", abs(_omega_with_ego_yaw(-0.3)) < 0.02,
      f"omega={_omega_with_ego_yaw(-0.3):+.3f}")

# ---------------------------------------------------------------- 
print()
print(f"{len(failures) and 'FAILED: ' + ', '.join(failures) or '모든 검사 통과'} "
      f"({len(failures)} 실패)")
sys.exit(1 if failures else 0)
