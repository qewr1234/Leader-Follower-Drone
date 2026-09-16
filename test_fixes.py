#!/usr/bin/env python3
"""C1~C6 + H2 회귀 테스트.

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
    MAV_CMD_NAV_LAND = 21


for _name in ("cv2", "pyrealsense2", "serial"):
    _stub(_name)
_stub("ultralytics", YOLO=object)
_stub("pymavlink")
_stub("pymavlink.mavutil", mavlink=_FakeMavlinkConsts, mavutil=None)
sys.modules["pymavlink"].mavutil = sys.modules["pymavlink.mavutil"]

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
for _ in range(60):                     # 30fps 2초
    ek.predict(1 / 30)
    ek.update_bearing2d([0.0, 0.0], Rb)
check("rcoast: bearing-only는 coast_time을 되돌린다 (기존 동작)",
      ek.coast_time == 0.0 and ek.is_reliable())
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

# ---------------------------------------------------------------- 
print()
print(f"{len(failures) and 'FAILED: ' + ', '.join(failures) or '모든 검사 통과'} "
      f"({len(failures)} 실패)")
sys.exit(1 if failures else 0)
