#!/usr/bin/env python3
"""C1~C6 + H2 회귀 테스트 — 단위 검사 97개.

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


class _Master0(_Master):
    """pymavlink 2.4.4x 실제 동작: heartbeat 뒤에도 target_component 가 0 (SITL 에서 FC=? 로 재현됨)."""
    target_system, target_component = 1, 0


_m0 = _Master0([_HB(1, 1, "LOITER")])
mavlink_io.drain_messages(_m0)
check("HB: target_component=0 (pymavlink 2.4.4x) 이어도 FC heartbeat 를 받아들이고 컴포넌트를 1 로 고정",
      mavlink_io.get_vehicle_state()["mode"]["name"] == "LOITER" and _m0.target_component == 1,
      f"mode={mavlink_io.get_vehicle_state()['mode']['name']} comp={_m0.target_component}")
_hb_cam = _HB(1, 100, "Mode(0)", mav_type=2); _hb_cam.autopilot = 8          # 카메라: autopilot INVALID
mavlink_io.drain_messages(_Master0([_hb_cam]))
check("HB: target_component=0 이어도 autopilot=INVALID 인 컴포넌트는 무시", mavlink_io.get_vehicle_state()["mode"]["name"] == "LOITER")
mavlink_io.drain_messages(_Master([_HB(1, 1, "GUIDED")]))                    # 다음 검사들의 전제 복원
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
# 쓰는 스트림(POSITION/EXTRA1/EXTENDED_STATUS)만 요청한다 — ALL(0) 을 요청하면 RAW_SENS/EXTRA2 까지
# 매 프레임 파싱하게 된다.
check("FC: 대기 메시지(10s, 20s) 출력 + 연결 후 필요한 data stream 3개만 요청 (ALL 제외)",
      "heartbeat 대기 중 (10s)" in _out and "heartbeat 대기 중 (20s)" in _out
      and "connected" in _out and len(_fake_fc.stream_calls) == 3
      and all(c[:2] == (1, 7) for c in _fake_fc.stream_calls)
      and all(c[2] != 0 for c in _fake_fc.stream_calls),
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
check("하네스: --all이 depth_loss·handover·leader_sine까지 포함",
      '"depth_loss", "handover", "leader_sine"] if ARGS.all' in _h)

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

# ------------------------------------------------- 2차 리팩토링 회귀
# detector 후처리: boxes.data 행렬 → ROI 오프셋·클립·클래스 필터·정렬
from detector import _postprocess  # noqa: E402

_names = {0: "person", 1: "leader_drone"}
_data = np2.array([
    [10.0, 20.0, 50.0, 60.0, 0.30, 1],     # leader_drone, 작은 상자
    [5.0, 5.0, 100.0, 100.0, 0.90, 0],     # person → 필터
    [0.0, 0.0, 200.0, 150.0, 0.30, 1],     # leader_drone, 같은 conf 더 큰 면적 → 먼저
    [-20.0, -20.0, 30.0, 30.0, 0.95, 1],   # 음수 좌표 → 클립
])
_dets = _postprocess(_data, 100, 50, 640, 480, _names, "leader_drone")
check("detector: 대상 클래스만 남고 (conf, area) 내림차순", [d["conf"] for d in _dets] == [0.95, 0.30, 0.30]
      and _dets[1]["area"] > _dets[2]["area"], f"{[(d['conf'], d['area']) for d in _dets]}")
check("detector: ROI 오프셋 적용 + 클립", _dets[0]["bbox"] == (80, 30, 130, 80) and _dets[2]["bbox"] == (110, 70, 150, 110),
      f"{[d['bbox'] for d in _dets]}")
check("detector: dict 키", set(_dets[0]) == {"bbox", "conf", "cls", "name", "area"} and _dets[0]["name"] == "leader_drone")
_d2 = YoloDetector(model=_FakeModel(), target_class_name="leader_drone")
check("detector: predict 없는 스텁 모델도 생성·warmup 통과 (imgsz=config, classes id 역조회)",
      _d2.imgsz == CONFIG["detector"]["imgsz"] and _d2.target_cls_id == 0 and _d2.warmup(640, 480) is None)


# 회귀 방지: ultralytics Model.predict 는 {**overrides, **custom(conf=0.25), **kwargs} 로 병합하므로
# conf 를 overrides 에만 넣으면 0.25 로 덮인다. conf 가 매 호출 kwargs 로 도달해야 한다.
class _FakeUltraModel:
    names = {0: "leader_drone"}

    def __init__(self):
        self.overrides = {}
        self.calls = []

    def predict(self, img, **kw):
        self.calls.append(kw)
        return []


_fm = _FakeUltraModel()
with contextlib.redirect_stdout(_io.StringIO()):
    _d3 = YoloDetector(model=_fm, conf_thres=0.6, target_class_name="leader_drone")
    _d3.detect(np2.zeros((48, 64, 3), dtype=np2.uint8))
check("detector: conf/iou/imgsz/classes 가 overrides 가 아닌 predict kwargs 로 매 호출 전달 (conf 0.25 덮어쓰기 회피)",
      len(_fm.calls) == 1 and _fm.calls[0].get("conf") == 0.6 and _fm.calls[0].get("classes") == [0]
      and "iou" in _fm.calls[0] and "imgsz" in _fm.calls[0] and "conf" not in _fm.overrides,
      f"calls={_fm.calls} overrides={_fm.overrides}")

# logger: flatten 결과가 이전과 같은가 (키 이름, ';' 조인, 6g 포맷, numpy 스칼라 변환)
from logger import ExperimentLogger  # noqa: E402
_row = {"a": 1, "b": {"c": np2.float64(1.5), "d": np2.int64(3), "e": np2.bool_(True), "f": None},
        "g": [1, np2.float32(2.5), "x"], "h": np2.array([[1.23456789, 2.0]]), "i": (True, False), "s": "str", "z": 0.1}
_flat = ExperimentLogger._flatten(_row)
check("logger: 평탄화 결과 동일",
      _flat == {"a": 1, "b.c": 1.5, "b.d": 3, "b.e": True, "b.f": None, "g": "1;2.5;x", "h": "1.23457;2", "i": "True;False", "s": "str", "z": 0.1}
      and type(_flat["b.c"]) is float and type(_flat["b.d"]) is int and type(_flat["b.e"]) is bool, f"{_flat}")

# measurement: 큰 ROI 서브샘플은 median 을 거의 바꾸지 않고, 작은 ROI 는 건드리지 않는다
import measurement as _meas_mod  # noqa: E402
_mb2 = MeasurementBuilder({"fx": 384.0, "fy": 384.0, "ppx": 320.0, "ppy": 240.0}, depth_scale=0.001)
_rng = np2.random.default_rng(0)
_dimg = (4000 + _rng.normal(0, 30, size=(480, 640))).astype(np2.uint16)
_big = _mb2._depth_stats(_dimg, (100, 60, 500, 420))           # inner 220x198 > 20000px → 서브샘플
_saved = _meas_mod.SUBSAMPLE_ABOVE_PX; _meas_mod.SUBSAMPLE_ABOVE_PX = 10**9
_big_full = _mb2._depth_stats(_dimg, (100, 60, 500, 420))
_meas_mod.SUBSAMPLE_ABOVE_PX = _saved
check("measurement: 큰 ROI 서브샘플 전후 depth_m 차이 < 1cm",
      abs(_big["depth_m"] - _big_full["depth_m"]) < 0.01 and _big["depth_valid_count"] * 3 < _big_full["depth_valid_count"],
      f"sub={_big['depth_m']:.4f} full={_big_full['depth_m']:.4f} n={_big['depth_valid_count']}/{_big_full['depth_valid_count']}")
_small = _mb2._depth_stats(_dimg, (300, 220, 390, 310))
_meas_mod.SUBSAMPLE_ABOVE_PX = 10**9
_small_full = _mb2._depth_stats(_dimg, (300, 220, 390, 310))
_meas_mod.SUBSAMPLE_ABOVE_PX = _saved
check("measurement: 90x90 ROI 는 서브샘플 없음 (정확히 같은 값)", _small == _small_full)

# scheduler: hover 계층(p_cv>0.75) 삭제 — p_cv 가 아무리 커도 normal_detect_every 이상으로 건너뛰지 않는다
from scheduler import PerceptionScheduler  # noqa: E402
_de = PerceptionScheduler._choose_detect_period(img_unc=5.0, p_ct=0.05, lost_count=0, cfg=CONFIG["scheduler"])
check("scheduler: 안정 상태 검출 주기 = normal_detect_every (hover 계층 없음)",
      _de == CONFIG["scheduler"]["normal_detect_every"] and "hover_detect_every" not in CONFIG["scheduler"], f"detect_every={_de}")

# imm_ekf: get_state() 캐시가 상태 변경(predict/update/compensate/속도 힌트) 뒤 갱신되는가
from leader_telemetry import apply_leader_velocity_hint_to_imm  # noqa: E402
ek3 = ImmEkf(); ek3.init([0.0, 0.0, 5.0])
x0 = ek3.get_state()[0].copy()
ek3.predict(0.1); x1 = ek3.get_state()[0].copy()
ek3.update_position3d([0.2, 0.0, 5.0]); x2 = ek3.get_state()[0].copy()
ek3.compensate_ego_yaw(0.1); x3 = ek3.get_state()[0].copy()
apply_leader_velocity_hint_to_imm(ek3, [1.0, 0.0, 0.0], alpha=0.5); x4 = ek3.get_state()[0].copy()
check("ekf: 캐시가 update/compensate/속도 힌트 뒤 갱신됨",
      not np3.allclose(x1, x2) and not np3.allclose(x2, x3) and not np3.allclose(x3, x4) and abs(x4[3] - 0.5 * x3[3] - 0.5) < 1e-9,
      f"vx: {x3[3]:.3f} → {x4[3]:.3f}")
check("ekf: 같은 상태에서 두 번 부르면 같은 객체 (캐시 적중)", ek3.get_state()[0] is ek3.get_state()[0])

# ------------------------------------------------- 자세(roll/pitch/yaw) 보정 — 돌풍에 기운 기체가 리더 이동으로 보이지 않게
from main import ego_rotation_cam, rot_body_to_ned, _CAM_FROM_BODY  # noqa: E402

_d = 0.2; _c, _s = _math.cos(_d), _math.sin(_d)
_T_yaw = ego_rotation_cam((0.0, 0.0, 0.0), (0.0, 0.0, _d))
check("자세보정: yaw 만 바뀌면 기존 compensate_ego_yaw 행렬과 동일",
      np3.allclose(_T_yaw, [[_c, 0, -_s], [0, 1, 0], [_s, 0, _c]], atol=1e-12))
_ahead = np3.array([0.0, 0.0, 5.0])                                      # 정면 5m (카메라 z)
_p = ego_rotation_cam((0.0, 0.0, 0.0), (0.0, _math.radians(10), 0.0)) @ _ahead
check("자세보정: 기수 10° 들리면 정면 타겟이 영상에서 아래(+y)로 내려감",
      _p[1] > 0.8 and abs(_p[0]) < 1e-9 and abs(_p[1] - 5 * _math.sin(_math.radians(10))) < 1e-9, f"p={_p.round(3)}")
_p = ego_rotation_cam((0.0, 0.0, 0.0), (_math.radians(10), 0.0, 0.0)) @ np3.array([5.0, 0.0, 0.0])
check("자세보정: 우측 롤 10° 이면 우측 타겟이 영상에서 위(-y)로 올라감, 정면 타겟은 불변",
      _p[1] < -0.8 and abs(_p[0] - 5 * _math.cos(_math.radians(10))) < 1e-9
      and np3.allclose(ego_rotation_cam((0.0, 0.0, 0.0), (_math.radians(10), 0.0, 0.0)) @ _ahead, _ahead), f"p={_p.round(3)}")
_a, _b, _cc = (0.1, -0.2, 1.0), (-0.3, 0.25, 1.4), (0.05, 0.4, -2.0)
check("자세보정: 왕복은 항등, 합성은 결합적",
      np3.allclose(ego_rotation_cam(_a, _b) @ ego_rotation_cam(_b, _a), np3.eye(3), atol=1e-12)
      and np3.allclose(ego_rotation_cam(_a, _cc), ego_rotation_cam(_b, _cc) @ ego_rotation_cam(_a, _b), atol=1e-12))


def _sim_attitude(compensate, T_end=10.0, v=1.0):
    """리더: 북쪽 5m 앞에서 동쪽으로 v m/s 직진(참 선회율 0). 팔로워: yaw 0.2rad/s + pitch ±10°(0.5Hz) + roll ±8°(0.7Hz).
    반환: (월드 속도 추정 오차[m/s], CT omega)."""
    ek = ImmEkf(); dt = 1 / 30; Rm = np3.diag([0.15 ** 2, 0.15 ** 2, 0.25 ** 2]); prev = None
    for k in range(int(T_end / dt) + 1):
        t = k * dt
        rpy = (_math.radians(8) * _math.sin(2 * _math.pi * 0.7 * t), _math.radians(10) * _math.sin(2 * _math.pi * 0.5 * t), 0.2 * t)
        R = rot_body_to_ned(*rpy)
        z = _CAM_FROM_BODY @ R.T @ np3.array([5.0, v * t, 0.0])
        if not ek.initialized:
            ek.init(z)
        else:
            ek.predict(dt)
            if compensate and prev is not None:
                ek.compensate_ego_rotation(ego_rotation_cam(prev, rpy))
            ek.update_position3d(z, Rm)
        prev = rpy
    x = ek.get_state()[0]
    v_w = R @ _CAM_FROM_BODY.T @ x[3:6]
    return float(np3.linalg.norm(v_w - [0.0, v, 0.0])), float(ek.filters[1]._omega)


_e_on, _w_on = _sim_attitude(True)
_e_off, _w_off = _sim_attitude(False)
check("자세보정: yaw+pitch+roll 동시 요동 중에도 월드 속도 오차 < 0.15 m/s, CT omega < 0.05",
      _e_on < 0.15 and abs(_w_on) < 0.05, f"err={_e_on:.3f} omega={_w_on:+.3f}")
check("자세보정: 보정 없이는 같은 시나리오에서 오차가 더 큼 (회귀 방지용 대조)",
      _e_off > _e_on * 2, f"err on/off = {_e_on:.3f}/{_e_off:.3f}")

# ------------------------------------------------- camera: 실외 노출 옵션·AE 측광 ROI (pyrealsense2 스텁)
import camera as _camera  # noqa: E402
_rs = sys.modules["pyrealsense2"]
_rs.option = types.SimpleNamespace(**{n: n for n in ("enable_auto_exposure", "auto_exposure_priority", "backlight_compensation",
                                                     "auto_exposure_limit", "auto_exposure_limit_toggle", "exposure")})
_rs.camera_info = types.SimpleNamespace(name="name")
_rs.pipeline = _rs.config = lambda: None


class _FakeRoi:
    pass


_rs.region_of_interest = _FakeRoi


class _FakeColorSensor:
    def __init__(self, supported, fail_roi=False):
        self.supported, self.set, self.rois, self.fail_roi = set(supported), [], [], fail_roi
    def supports(self, opt): return opt in self.supported
    def get_option_range(self, opt): return types.SimpleNamespace(min=0.0, max=10000.0 if opt in ("exposure", "auto_exposure_limit") else 1.0)
    def set_option(self, opt, v): self.set.append((opt, v))
    def as_roi_sensor(self): return self
    def set_region_of_interest(self, r):
        if self.fail_roi:
            raise RuntimeError("busy")
        self.rois.append((r.min_x, r.min_y, r.max_x, r.max_y))


_cfg_cam = {"color_auto_exposure_priority": False, "color_exposure_max_us": 8000, "color_backlight_compensation": True}
_sn = _FakeColorSensor(vars(_rs.option).keys())
_camera.apply_color_exposure_options(_sn, _cfg_cam)
check("camera: AE priority off·역광보정 on·노출 상한 8000µs → 80 (RGB 100µs 단위)",
      ("auto_exposure_priority", 0.0) in _sn.set and ("backlight_compensation", 1.0) in _sn.set
      and ("auto_exposure_limit", 80.0) in _sn.set and ("enable_auto_exposure", 1.0) in _sn.set, f"set={_sn.set}")
_sn_old = _FakeColorSensor(["enable_auto_exposure", "auto_exposure_priority", "backlight_compensation"])
_camera.apply_color_exposure_options(_sn_old, _cfg_cam)
check("camera: 구버전(auto_exposure_limit 없음)도 예외 없이 나머지 옵션 적용",
      len(_sn_old.set) == 3 and not any(o == "auto_exposure_limit" for o, _ in _sn_old.set), f"set={_sn_old.set}")

_r = _camera.roi_from_bbox((300, 200, 340, 230), 640, 480)
check("camera: AE ROI 는 bbox 1.5배를 프레임 안에서, 최소 32px",
      _r == (290, 192, 350, 237) and _camera.roi_from_bbox(None, 640, 480) == (0, 0, 639, 479)
      and _camera.roi_from_bbox((0, 0, 4, 4), 640, 480)[2:] >= (32, 32)
      and _camera.roi_from_bbox((630, 470, 639, 479), 640, 480) == (607, 447, 639, 479), f"roi={_r}")

_cam = _camera.D435i(); _cam._color_sensor = _FakeColorSensor(vars(_rs.option).keys())
_sent = [_cam.set_exposure_roi((300, 200, 340, 230), 0.0),     # 첫 bbox → 전송
         _cam.set_exposure_roi((400, 200, 440, 230), 0.5),     # 1초 안 → 억제
         _cam.set_exposure_roi((320, 210, 360, 240), 1.5),     # 중심 이동 20px < 64px → 억제
         _cam.set_exposure_roi((420, 200, 460, 230), 1.6),     # 120px 이동 → 전송
         _cam.set_exposure_roi(None, 2.0),                     # 소실: 1초 안 → 억제
         _cam.set_exposure_roi(None, 3.0),                     # 소실: 전체 프레임 → 전송
         _cam.set_exposure_roi(None, 9.0)]                     # 같은 ROI → 억제
check("camera: AE ROI 는 1Hz·10% 이동·소실 시 전체 복귀 규칙대로만 전송",
      _sent == [True, False, False, True, False, True, False] and len(_cam._color_sensor.rois) == 3
      and _cam._color_sensor.rois[-1] == (0, 0, 639, 479), f"sent={_sent} rois={_cam._color_sensor.rois}")
_cam2 = _camera.D435i(); _cam2._color_sensor = _FakeColorSensor(vars(_rs.option).keys(), fail_roi=True)
_ok1 = _cam2.set_exposure_roi((300, 200, 340, 230), 0.0); _ok2 = _cam2.set_exposure_roi((300, 200, 340, 230), 0.5)
_cam2._color_sensor.fail_roi = False; _ok3 = _cam2.set_exposure_roi((300, 200, 340, 230), 1.1)
check("camera: ROI 설정 실패는 예외 없이 False, 1초 뒤 재시도해 성공",
      (_ok1, _ok2, _ok3) == (False, False, True), f"{(_ok1, _ok2, _ok3)}")
_cam3 = _camera.D435i()
check("camera: 컬러 센서 없으면(하네스) set_exposure_roi 는 조용히 False", _cam3.set_exposure_roi((0, 0, 10, 10), 0.0) is False)

# ------------------------------------------------- 리더 속도 피드포워드
_kff = main.KFF_LEADER_VEL
_c0 = main.compute_velocity_cmd_from_estimate([3.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0)
_c1 = main.compute_velocity_cmd_from_estimate([3.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0, None, [0.3, 0.1, 0.05])
check("FF: 목표 거리(오차 0)에서 리더 속도만큼 명령 KFF·v — FRU up 은 BODY_NED vz 부호 반전, 인자 없으면 기존과 동일(0)",
      np3.allclose(_c0[:3], 0.0) and abs(_c1[0] - _kff * 0.3) < 1e-9 and abs(_c1[1] - _kff * 0.1) < 1e-9
      and abs(_c1[2] + _kff * 0.05) < 1e-9, f"cmd={_c1.round(3)}")
_vf = main.follower_velocity_fru({"local_position": {"vx": 1.0, "vy": 0.0, "vz": -0.2}, "attitude": {"yaw": _math.pi / 2}})
check("FF: 자기 속도 NED→FRU (동쪽을 보며 북진 = 좌측 이동, vz 부호 반전), 값 없으면 None",
      np3.allclose(_vf, [0.0, -1.0, 0.2], atol=1e-12) and main.follower_velocity_fru({"local_position": {}, "attitude": {}}) is None,
      f"fru={_vf.round(3)}")
_db = main.FF_DEADBAND_MPS
_f_small = main.leader_velocity_ff([0, 0, 0], [0.5 * _db, 0, 0], 100.0)
_f_half = main.leader_velocity_ff([0, 0, 0], [1.5 * _db, 0, 0], 100.0)
_f_3db = main.leader_velocity_ff([0, 0, 0], [3.0 * _db, 0, 0], 100.0)
_f_tau = main.leader_velocity_ff([0, 0, 0], [0.3, 0, 0], main.FF_TAU_SEC)
_f_none = main.leader_velocity_ff([0.3, 0, 0], None, 100.0)
check("FF: 소프트 데드존(0.5·db→0, 1.5·db→0.5·db, 3·db→2·db: 기울기 1), 시정수 τ 에서 63% (τ=2.0s), None 이면 0 으로 감쇠",
      _f_small[0] == 0.0 and abs(_f_half[0] - 0.5 * _db) < 1e-9 and abs(_f_3db[0] - 2.0 * _db) < 1e-9
      and abs(_f_tau[0] - (0.3 - _db) * (1 - _math.exp(-1))) < 1e-9 and abs(main.FF_TAU_SEC - 2.0) < 1e-9
      and abs(_f_none[0]) < 1e-9, f"{_f_small[0]:.3f} {_f_half[0]:.3f} {_f_3db[0]:.3f} {_f_tau[0]:.3f} {_f_none[0]:.3f}")
_sl0 = main.self_velocity_lpf(None, [0.3, 0, 0], 0.1)
_sl1 = main.self_velocity_lpf([0, 0, 0], [0.3, 0, 0], main.FF_SELF_TAU_SEC)
check("FF: 자기 속도 정합 저역통과 — 첫 샘플은 그대로, 시정수(0.3s = EKF 속도 지연 실측)에서 63%",
      np3.allclose(_sl0, [0.3, 0, 0]) and abs(_sl1[0] - 0.3 * (1 - _math.exp(-1))) < 1e-9 and abs(main.FF_SELF_TAU_SEC - 0.3) < 1e-9,
      f"{_sl0[0]:.3f} {_sl1[0]:.3f}")


def _sim_1d(kff, v_l=0.3, tau_fc=0.3, delta_ekf=1.0, T=40.0):
    """1축 폐루프: FC 속도루프 1차 지연 τ, EKF 상대속도 1차 지연 δ(최악 1s), 리더 v_l 등속. (정상상태 오차, 최대 |오차|) 반환."""
    saved = main.KFF_LEADER_VEL; main.KFF_LEADER_VEL = kff
    dt = 1 / 30; front, v_f, v_rel_est = 3.0, 0.0, 0.0; ff = np3.zeros(3); cmd = np3.zeros(4); e_max = 0.0; v_self = None
    try:
        for _ in range(int(T / dt)):
            v_rel = v_l - v_f
            v_rel_est += (v_rel - v_rel_est) * (dt / delta_ekf)
            v_self = main.self_velocity_lpf(v_self, [v_f, 0.0, 0.0], dt)
            ff = main.leader_velocity_ff(ff, [v_self[0] + v_rel_est, 0.0, 0.0], dt)
            u = main.compute_velocity_cmd_from_estimate([front, 0, 0], [v_rel_est, 0, 0], 1.0, None, ff)
            cmd = main.smooth_velocity_cmd(cmd, u, alpha=0.28, dt=dt)
            v_f += (cmd[0] - v_f) * (dt / tau_fc)
            front += v_rel * dt
            e_max = max(e_max, abs(front - 3.0))
    finally:
        main.KFF_LEADER_VEL = saved
    return front - 3.0, e_max


_e_ff, _emax_ff = _sim_1d(_kff)
_e_p, _ = _sim_1d(0.0)
_e_theory_ff, _e_theory_p = (0.3 - _kff * (0.3 - _db)) / main.KP_FORWARD, 0.3 / main.KP_FORWARD
check("FF: 1축 폐루프(FC τ=0.3s, EKF 지연 1s) 정상상태 오차 = (v − KFF·(v−DB))/Kp (소프트 데드존), P 만이면 v/Kp — 발산·진동 없음",
      abs(_e_ff - _e_theory_ff) < 0.05 and abs(_e_p - _e_theory_p) < 0.05 and _emax_ff < 1.0,
      f"ff={_e_ff:.2f}m(이론 {_e_theory_ff:.2f}) p={_e_p:.2f}m(이론 {_e_theory_p:.2f}) max|e|={_emax_ff:.2f}")

# ------------------------------------------------- 미션: 리더 절대 속도(자기 속도 + 상대 속도) 기준
_mv = dict(rel_est=[3.0, 0.0, 0.0], leader_alt=50.0, pos_cov_trace=1.0)
m20 = MissionManager(); t = 0.0
for _ in range(12):                              # 리더 출발 → FOLLOW 확정
    st, _ = m20.update(now=t, leader_visible=True, rel_vel_est=[0.3, 0, 0], leader_vel_body=[0.3, 0, 0], **_mv); t += 0.1
check("미션: 출발 확인 후 FOLLOW", st == S_FOLLOW, f"state={st}")
st, _ = m20.update(now=t, leader_visible=True, rel_vel_est=[0.0, 0, 0], leader_vel_body=[0.3, 0, 0], **_mv); t += 0.1
check("미션: 따라잡아 상대 속도 0 이어도 리더 절대 속도 0.3 이면 FOLLOW 유지", st == S_FOLLOW, f"state={st}")
st, _ = m20.update(now=t, leader_visible=True, rel_vel_est=[-0.3, 0, 0], leader_vel_body=[0.0, 0, 0], **_mv); t += 0.1
check("미션: 리더 정지(후미는 아직 이동, 상대 -0.3) → 절대 속도 0 이라 LEADER_HOVER", st == S_LEADER_HOVER, f"state={st}")
st, _ = m20.update(now=t, leader_visible=True, rel_vel_est=[0.5, 0, 0], leader_vel_body=[0.5, 0, 0], leader_vel_world=[0.0, 0, 0], **_mv); t += 0.1
check("미션: ESP32 절대 속도가 있으면 최우선 (0 → LEADER_HOVER 유지)", st == S_LEADER_HOVER, f"state={st}")
st, _ = m20.update(now=t, leader_visible=True, rel_vel_est=[0.5, 0, 0], **_mv); t += 0.1
check("미션: 둘 다 없으면 상대 속도 폴백 (0.5 → FOLLOW)", st == S_FOLLOW, f"state={st}")
m21 = MissionManager(); t = 0.0
for _ in range(25):                              # 후미가 하강 중이면 상대 vz 는 +, 리더 절대 vz 는 -0.5 (진짜 착륙)
    st, p = m21.update(now=t, leader_visible=True, rel_est=[3.0, 0, 0], rel_vel_est=[0.0, 0, +0.3],
                       leader_vel_body=[0.0, 0, -0.5], leader_alt=0.3, pos_cov_trace=1.0); t += 0.1
check("미션: 착륙 판정도 절대 vz 로 (상대 vz 가 + 여도 리더가 하강하면 CONFIRMED_LANDING)", p["land"] is True, f"state={st}")

# ------------------------------------------------- mavlink_io: 수신율 진단
mavlink_io._rate_t0 = None; mavlink_io._rate_counts.clear()
_t = 1000.0
check("rx: 첫 호출은 기준 시각만 잡음", mavlink_io.stream_rates_text(_t) == "rx[Hz] -")
mavlink_io.drain_messages(_Master([_HB(1, 1, "GUIDED") for _ in range(2)] + [_SysStatus(12000)]))
_rx = mavlink_io.stream_rates_text(_t + 2.0)
check("rx: 2초에 HB 2개 → HB=1, 나머지 0, 호출 뒤 카운터 리셋",
      _rx == "rx[Hz] HB=1 LP=0 ATT=0 GP=0" and mavlink_io.stream_rates_text(_t + 3.0) == "rx[Hz] HB=0 LP=0 ATT=0 GP=0", _rx)

# ------------------------------------------------- 분석: 바깥 루프 안정성 여유 (docs/STABILITY_MARGINS.md)
from analysis import stability_margins as _sm  # noqa: E402
_frf = _sm.EkfFrf(_sm.load_frf())               # docs/stability_margins.json 의 IMM-EKF 실측 주파수응답 (없으면 재실측 ~40s)
_lag = _sm.ekf_velocity_step_lag()
check("분석: IMM-EKF 속도 추정 63% 응답 ≤ 0.5s, 램프에 위치 지연 없음",
      _lag["t63_velocity_s"] is not None and _lag["t63_velocity_s"] <= 0.5 and abs(_lag["pos_lag_m_at_end"]) < 0.01,
      f"t63={_lag['t63_velocity_s']}s lag={_lag['pos_lag_m_at_end']:.4f}m")


def _margins_of(**kw):
    p = _sm.Params(**kw)
    r = _sm.margins(_sm.open_loop(p, _sm.W, _frf), _sm.W)
    r.update(_sm.string_stability(p, _sm.W, _frf))
    r["self_fb_peak"] = float(np3.max(np3.abs(_sm.self_feedback_path(p, _sm.W, _frf))))
    return r


_r0 = _margins_of(kff=0.0)
check("분석: P+D 기준선(KFF=0) 전후축 PM ≥ 45°, GM ≥ 6dB, Ms ≤ 1.5, |Γ| ≤ 1 (스트링 안정)",
      _r0["pm_deg"] >= 45 and _r0["gm_db"] >= 6 and _r0["Ms"] <= 1.5 and _r0["peak"] <= 1.0 + 1e-3,
      f"PM={_r0['pm_deg']:.1f} GM={_r0['gm_db']:.1f} Ms={_r0['Ms']:.2f} Γ={_r0['peak']:.3f}")
_rb = _margins_of(tau_ff=0.7, tau_m=0.0)
check("분석 골든: 수정 전 설계(FF τ 0.7s, 자기 속도 정합 없음) 전후축 GM 4.9dB·Ms 2.3·|Γ| 피크 1.80 @1.15rad/s — 결함의 기록 (회귀 방지용 대조)",
      abs(_rb["gm_db"] - 4.9) < 0.3 and abs(_rb["Ms"] - 2.31) < 0.1 and abs(_rb["peak"] - 1.80) < 0.05 and abs(_rb["w_peak"] - 1.15) < 0.1,
      f"GM={_rb['gm_db']:.2f} Ms={_rb['Ms']:.2f} Γ={_rb['peak']:.3f}@{_rb['w_peak']:.2f}")
_rr = _margins_of()                      # 코드 값 (τ_ff 2.0, τ_m 0.3)
check("분석: 현재 설계(코드 값: FF τ 2.0s + 자기 속도 정합 LPF 0.3s) 전 축 PM ≥ 45°, GM ≥ 12dB, Ms ≤ 1.35, |Γ| 피크 ≤ 1.15, 양성 되먹임 경로 피크 ≤ 0.15",
      all(_margins_of(kp=kp, kd=kd)["gm_db"] >= 12 and _margins_of(kp=kp, kd=kd)["pm_deg"] >= 45
          for kp, kd in ((main.KP_RIGHT, main.KD_RIGHT), (main.KP_UP, main.KD_UP)))
      and _rr["gm_db"] >= 12 and _rr["Ms"] <= 1.35 and _rr["peak"] <= 1.15 and _rr["self_fb_peak"] <= 0.15 and abs(_rr["dc_gain"] - 1.0) < 0.01,
      f"GM={_rr['gm_db']:.1f} Ms={_rr['Ms']:.2f} Γ={_rr['peak']:.3f} fb={_rr['self_fb_peak']:.2f}")

# ------------------------------------------------- 추적성: docs/REQUIREMENTS.md 의 참조가 코드와 맞는가
from analysis import trace_check as _tc  # noqa: E402
_ts = _tc.run(verbose=False)
check("추적성: REQUIREMENTS.md 의 검증 근거(UT/CL/SITL/AN/INSPECT)가 전부 실제로 존재, 요구도 ≥ 50개, 폐루프 검사 전부 요구도에 연결",
      not _ts["errors"] and _ts["requirements"] >= 50 and not _ts["cl_orphans"],
      "; ".join(_ts["errors"][:3]) or f"{_ts['requirements']}개, UT {_ts['ut_traced']}/{_ts['ut_total']}, CL {_ts['cl_traced']}/{_ts['cl_total']}")

# ------------------------------------------------- 편대 토대 (formation.py) — 선두 1 : 후미 N (docs/MULTI_FOLLOWER_FOUNDATION.md)
import math as _math  # noqa: E402
import formation as _fm  # noqa: E402
from leader_telemetry import LeaderTelemetryReceiver as _LTR  # noqa: E402

_rng = np3.random.default_rng(7)
_same = True
for _ in range(50):
    _rel = _rng.uniform(-5, 5, 3)
    _e, _deg = _fm.slot_error_fru(_rel, _fm.los_slot(main.TARGET_DISTANCE_M))
    _old = [_rel[0] - main.TARGET_DISTANCE_M, _rel[1], _rel[2]]
    _same &= (not _deg) and all(float(a) == float(b) for a, b in zip(_e, _old))
_c_old = main.compute_velocity_cmd_from_estimate([3.7, -0.4, 0.2], [0.1, 0.0, 0.0], 1.0, main.TARGET_DISTANCE_M, [0.3, 0.0, 0.0])
_c_new = main.compute_velocity_cmd_from_estimate([3.7, -0.4, 0.2], [0.1, 0.0, 0.0], 1.0, None, [0.3, 0.0, 0.0],
                                                 slot_error=_fm.slot_error_fru([3.7, -0.4, 0.2], _fm.los_slot(main.TARGET_DISTANCE_M))[0])
check("편대: 기본 LOS 슬롯 (−TARGET, 0, 0) 은 기존 오차 (front−TARGET, right, up) 와 비트 단위로 같고 명령도 같다 (기존 동작 보존)",
      _same and all(float(a) == float(b) for a, b in zip(_c_old, _c_new)), f"cmd={np3.round(_c_new, 4)}")

_slot = _fm.FormationSlot("right_wing", (-3.0, 2.0, 0.0), _fm.FRAME_LEADER, follower_id="F2")
_e, _deg = _fm.slot_error_fru([5.0, 0.0, 0.0], _slot, rel_heading=_math.pi / 2)
check("편대: 리더 heading 기준 슬롯 — 리더가 내 기준 +90°(동쪽)를 볼 때 '리더 뒤 3 m·우측 2 m' 는 내 FRU 로 (−2, −3) → 오차 (3, −3, 0)",
      np3.allclose(_e, [3.0, -3.0, 0.0]) and not _deg, f"e={np3.round(_e, 3)}")
_e0, _ = _fm.slot_error_fru([5.0, 0.0, 0.0], _slot, rel_heading=0.0)
_ed, _deg = _fm.slot_error_fru([5.0, 0.0, 0.0], _slot, rel_heading=None)
check("편대: 상대 heading 0 이면 리더 프레임 슬롯은 LOS 와 같은 기하(오차 (2, 2, 0)); heading 을 모르면 같은 거리(3.61 m)의 LOS 후방으로 강등하고 알린다",
      np3.allclose(_e0, [2.0, 2.0, 0.0]) and _deg and np3.allclose(_ed, [5.0 - _math.hypot(3, 2), 0.0, 0.0]), f"e0={np3.round(_e0, 3)} ed={np3.round(_ed, 3)}")
_en, _ = _fm.slot_error_fru([0.0, 0.0, 0.0], _fm.FormationSlot("west3", (0.0, -3.0, 0.0), _fm.FRAME_NED), follower_yaw=_math.pi / 2)
check("편대: NED 슬롯 (북 0, 동 −3, 하 0) 은 동쪽을 보는 후미의 FRU 로 (−3, 0, 0) — ArduPilot FOLL_OFS_TYPE=0 대응",
      np3.allclose(_en, [-3.0, 0.0, 0.0]), f"e={np3.round(_en, 3)}")
_eg, _ = _fm.slot_error_fru([0.0, 0.0, 0.0], _fm.los_slot(3.0), min_distance=8.0)
_eg2, _ = _fm.slot_error_fru([0.0, 0.0, 0.0], _fm.FormationSlot("s", (-3.0, 4.0, 0.0), _fm.FRAME_LEADER), rel_heading=0.0, min_distance=10.0)
check("편대: 비전 거리 없이 GPS 뿐이면 슬롯 방향을 유지한 채 이격을 최소치로 늘린다 ((−3,0,0)→(−8,0,0) 정확히, (−3,4,0)→(−6,8,0)) — 기존 TARGET_DISTANCE_GPS_ONLY_M 규칙의 일반형",
      float(_eg[0]) == -8.0 and np3.allclose(_eg, [-8.0, 0.0, 0.0]) and np3.allclose(_eg2, [-6.0, 8.0, 0.0]), f"{_eg} {_eg2}")

_he = _fm.RelativeHeadingEstimator(min_speed_mps=0.5, hold_sec=2.0)
_h1 = _he.update(0.0, leader_yaw_ned=1.0, follower_yaw_ned=0.2, leader_vel_fru=[0.0, 1.0, 0.0])
_h2 = _he.update(1.0, leader_vel_fru=[0.0, 1.0, 0.0])
_h3 = _he.update(2.0, leader_vel_fru=[0.1, 0.1, 0.0])
_h4 = _he.update(5.0, leader_vel_fru=[0.1, 0.1, 0.0])
_h5 = _he.update(6.0, leader_yaw_ned=3.0, follower_yaw_ned=-3.0)
check("편대: 상대 heading 소스 우선순위 — 방송 yaw(0.8) > 속도 방향(+90°, ≥0.5 m/s) > 최근값 유지(2 s) > 없음(None), 각도는 ±π 로 감김",
      abs(_h1[0] - 0.8) < 1e-9 and _h1[1] == "broadcast" and abs(_h2[0] - _math.pi / 2) < 1e-9 and _h2[1] == "velocity"
      and abs(_h3[0] - _math.pi / 2) < 1e-9 and _h3[1] == "hold" and _h4 == (None, "none")
      and abs(_h5[0] - (6.0 - 2 * _math.pi)) < 1e-9 and _h5[1] == "broadcast", f"{_h1} {_h2} {_h3} {_h4} {_h5}")

_S = _fm.FormationSlot
_v_kw = dict(min_separation_m=2.0, depth_min_m=0.3, depth_max_m=10.0, depth_reserve_m=3.0)
_v_close = _fm.validate_formation([_S("a", (-3, 0, 0), "leader", "F1"), _S("b", (-3, 1.0, 0), "leader", "F2")], **_v_kw)
_v_far = _fm.validate_formation([_S("a", (-3, 0, 0), "leader", "F1"), _S("b", (-9, 0, 0), "leader", "F2")], **_v_kw)
_v_dup = _fm.validate_formation([_S("a", (-3, 0, 0), "leader", "F1"), _S("b", (-3, 2.5, 0), "leader", "F1")], **_v_kw)
_v_ok = _fm.validate_formation([_S("a", (-3, 0, 0), "leader", "F1"), _S("b", (-3, 2.5, 0), "leader", "F2"), _S("c", (-3, -2.5, 0), "leader", "F3")], **_v_kw)
check("편대: 유효성 — 슬롯 간 1 m(<2 m)·깊이창 여유 밖(9 m > 10−3)·follower_id 중복은 거부, V 자 3슬롯(2.5 m 간격)은 통과",
      len(_v_close) == 1 and len(_v_far) == 1 and len(_v_dup) == 1 and _v_ok == [], f"{_v_close} {_v_far} {_v_dup} {_v_ok}")
_cfg = {"follower_id": "F2", "slots": {"F2": {"offset": [-3, 2, 0], "frame": "leader", "slot_id": "rw"}}}
check("편대: config 슬롯 선택 — 내 follower_id 에 슬롯이 있으면 그것(리더 프레임), 없으면 None(LOS 기본 = 기존 동작)",
      _fm.slot_from_config(_cfg, "F2").slot_id == "rw" and _fm.slot_from_config(_cfg, "F2").frame == "leader"
      and _fm.slot_from_config(_cfg, "F9") is None and _fm.slot_from_config({}, "F1") is None)

_rx2 = _LTR(kind="udp", expected_leader_id="L1")
_acc = [_rx2._accept(parse_leader_json(js)) for js in ('{"lat":1,"lon":2,"alt":3,"leader_id":"L1"}',
                                                       '{"lat":1,"lon":2,"alt":3,"leader_id":"L2"}',
                                                       '{"lat":1,"lon":2,"alt":3}')]
_rx3 = _LTR(kind="udp", expected_leader_id="L1", require_leader_id=True)
check("편대: 리더 ID 필터(ArduPilot FOLL_SYSID 역할) — 기대 ID 만 채택, 다른 ID 는 버리고 셈, ID 없는 패킷은 호환을 위해 통과(require_leader_id 면 거부)",
      _acc == [True, False, True] and _rx2.dropped_other_leader == 1 and _rx2.latest_packet.leader_id == ""
      and not _rx3._accept(parse_leader_json('{"lat":1,"lon":2,"alt":3}')) and _rx3.latest_packet is None, f"{_acc}")
check("편대: 패킷에 yaw 가 없으면 None (0 = '북쪽' 으로 오해하지 않음), 있으면 float",
      parse_leader_json('{"lat":1,"lon":2,"alt":3}').yaw is None and parse_leader_json('{"lat":1,"lon":2,"alt":3,"yaw":0.3}').yaw == 0.3
      and build_leader_measurement_from_packet(_pk('{"lat":35.83,"lon":128.75,"alt":37.0}'), _FOLLOWER, now=0.0)["yaw"] is None)

_pkt = parse_leader_json('{"timestamp": 12.5, "lat": 35.83, "lon": 128.75, "alt_msl": 37.0, "vx": 1.0, "vy": 2.0, "vz": 0.5, "yaw": 0.3, "leader_id": "L1", "seq": 4}')
_ls = _fm.LeaderState.from_leader_packet(_pkt)
_ft = _ls.to_follow_target()
_back = _fm.LeaderState.from_follow_target(_ft, leader_id="L1", now=99.0)
check("편대: LeaderState ↔ MAVLink FOLLOW_TARGET(#144) 왕복 — lat/lon degE7 정수, timestamp ms, vel ENU→NED [north, east, down], yaw↔attitude_q, est_capabilities=pos|vel|att",
      _ft["lat"] == 358300000 and _ft["lon"] == 1287500000 and _ft["vel"] == [2.0, 1.0, -0.5] and _ft["timestamp"] == 12500
      and _ft["est_capabilities"] == (_fm.LeaderState.EST_POS | _fm.LeaderState.EST_VEL | _fm.LeaderState.EST_ATT)
      and abs(_back.yaw - 0.3) < 1e-9 and np3.allclose(_back.vel_enu, [1.0, 2.0, 0.5]) and _back.acc_enu is None
      and abs(_back.lat - 35.83) < 1e-7 and _back.leader_id == "L1" and abs(_back.timestamp - 12.5) < 1e-9, f"{_ft}")

# ------------------------------------------------- 편대: 체인 토폴로지 (선행기 추종 vs 선두 속도 방송) — analysis.chain_sim
_w = 0.35   # 현재 설계의 |Γ| 피크 부근 (docs/STABILITY_MARGINS.md 6절: 비선형 1단 1.158)
_ck = dict(n_followers=3, v_leader=0.30, T=60.0, profile="sine", omega=_w, amp=0.03)
_hp = _sm.chain_sim(**_ck)
_hb = _sm.chain_sim(topology="leader_broadcast", **_ck)
_ap = _sm.chain_sine_amplitudes(_hp, _w, t_from=24.0)["amplitudes"]
_ab = _sm.chain_sine_amplitudes(_hb, _w, t_from=24.0)["amplitudes"]
_gp = [a / _ap[0] for a in _ap[1:]]       # 리더 → n 단 누적 이득
_gb = [a / _ab[0] for a in _ab[1:]]
check("편대: 체인 토폴로지 — 선행기 추종(predecessor)은 리더→n 단 누적 이득이 단마다 커지지만, 선두 속도 방송(leader_broadcast)은 단 수와 무관하게 유지(≤ 1.15)·더 작다 (Seiler 2004 / Zheng 2016)",
      _gp[-1] > _gp[0] + 0.05 and _gb[-1] <= 1.15 and _gb[-1] <= _gb[0] + 0.03 and _gb[-1] < _gp[-1],
      f"predecessor={[round(g, 3) for g in _gp]} broadcast={[round(g, 3) for g in _gb]}")
_hk = _sm.chain_sim(n_followers=2, v_leader=0.3, T=40.0, profile="step", p=_sm.Params(kff=1.0), topology="leader_broadcast")
_sk = _sm.chain_summary(_hk, t_from=10.0)
_idx = int(np3.argmin(np3.abs(_hk["t"] - 28.0)))
_cruise = [float(e[_idx]) for e in _hk["err"]]
check("편대: 선두 속도 방송이면 KFF 1.0 도 안정(자기 속도 되먹임 경로 없음) — 최대 오차 < 0.8 m, 순항 잔차 ≈ KFF·DB/Kp = 0.23 m, 정지 후 0 (vision 소스의 KFF 1.0 은 선형 모델 GM −0.2 dB 로 불안정)",
      max(_sk["max_abs_err_m"]) < 0.8 and all(abs(e - 0.05 / main.KP_FORWARD) < 0.08 for e in _cruise) and all(abs(e) < 0.15 for e in _sk["final_err_m"]),
      f"max={[round(m, 2) for m in _sk['max_abs_err_m']]} cruise={[round(c, 2) for c in _cruise]} final={[round(f, 2) for f in _sk['final_err_m']]}")

# ------------------------------------------------- 수평화: 기체 기울기 → 제어 오차 편향 (main.level_fru_by_roll_pitch)
_rpy = (0.0, _math.radians(-10.0), 0.0)
_rel_t = main.camera_xyz_to_fru(main._CAM_FROM_BODY @ (main.rot_body_to_ned(*_rpy).T @ np3.array([3.0, 0.0, 0.0])))
_cmd_t = main.compute_velocity_cmd_from_estimate(_rel_t, np3.zeros(3), 1.0, 3.0, None)
_rel_l = main.level_fru_by_roll_pitch(_rel_t, _rpy[0], _rpy[1])
_cmd_l = main.compute_velocity_cmd_from_estimate(_rel_l, np3.zeros(3), 1.0, 3.0, None)
check("수평화: pitch −10° 로 기운 기체는 같은 고도 리더(3 m)에 vz −0.094 m/s(상한 0.12 의 78 %) 를 내지만 roll/pitch 를 되돌리면 오차 (0,0,0)·명령 0 — FC 는 BODY_NED 를 yaw 만 회전(ArduCopter body_to_earth2D, PX4 mavlink_receiver)",
      abs(_cmd_t[2] + 0.094) < 0.005 and abs(_rel_t[2] - 3 * _math.sin(_math.radians(10))) < 1e-6
      and np3.allclose(_rel_l, [3.0, 0.0, 0.0], atol=1e-9) and abs(_cmd_l[2]) < 1e-9 and abs(_cmd_l[0]) < 1e-9,
      f"tilted rel={np3.round(_rel_t, 3)} vz={_cmd_t[2]:+.3f} → leveled rel={np3.round(_rel_l, 3)}")
_rpy2 = (_math.radians(10.0), 0.0, 0.0)
_rel_t2 = main.camera_xyz_to_fru(main._CAM_FROM_BODY @ (main.rot_body_to_ned(*_rpy2).T @ np3.array([3.0, 2.0, 0.0])))
check("수평화: 우측 롤 10° 에 우측 2 m 리더는 카메라 프레임에서 위로 0.35 m 떠 보이지만 수평화하면 (3, 2, 0); 수평 기체면 항등",
      abs(_rel_t2[2] - 2 * _math.sin(_math.radians(10))) < 1e-6 and np3.allclose(main.level_fru_by_roll_pitch(_rel_t2, *_rpy2[:2]), [3.0, 2.0, 0.0], atol=1e-9)
      and np3.allclose(main.level_fru_by_roll_pitch([1.0, 2.0, 3.0], 0.0, 0.0), [1.0, 2.0, 3.0]), f"{np3.round(_rel_t2, 3)}")
check("수평화: 기본값은 꺼짐(config controller.level_by_attitude=False) — 기존 동작 보존, SITL 자세 시나리오 뒤 켤 것",
      main.LEVEL_BY_ATTITUDE is False and CONFIG["controller"]["level_by_attitude"] is False)

# ------------------------------------------------- ESP32 GPS 신선도: 팔로워 GLOBAL_POSITION_INT 가 오래되면 상대위치를 만들지 않는다
_stale_fs = {"global_position": {"lat": 358300000, "lon": 1287500000, "alt": 35000, "timestamp": 1.0}, "attitude": {"yaw": 0.0, "timestamp": 5.0}}
_js = '{"lat":35.83,"lon":128.75,"alt":37.0}'
_m_fresh = build_leader_measurement_from_packet(_pk(_js), _stale_fs, now=1.5, follower_gps_max_age_sec=0.7)
_m_stale = build_leader_measurement_from_packet(_pk(_js), _stale_fs, now=5.0, follower_gps_max_age_sec=0.7)
_m_nogate = build_leader_measurement_from_packet(_pk(_js), _stale_fs, now=5.0)
check("ESP32 GPS 신선도: 팔로워 GLOBAL_POSITION_INT 가 0.7 s 보다 오래됐으면 상대위치를 만들지 않고(stale_follower_gps), 게이트를 안 주면 기존과 같다",
      _m_fresh["available"] and not _m_stale["available"] and _m_stale["reason"] == "stale_follower_gps" and _m_nogate["available"],
      f"fresh={_m_fresh['available']} stale={_m_stale.get('reason')} nogate={_m_nogate['available']}")

# ---------------------------------------------------------------- 
print()
print(f"{len(failures) and 'FAILED: ' + ', '.join(failures) or '모든 검사 통과'} "
      f"({len(failures)} 실패)")
sys.exit(1 if failures else 0)
