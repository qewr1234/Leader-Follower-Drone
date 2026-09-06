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
state, policy = m2.update(now=106.0, leader_visible=False, **MOVING)
check("C1: 실제로 놓친 뒤에는 FAILSAFE_LAND 유지",
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

# ---------------------------------------------------------------- 
print()
print(f"{len(failures) and 'FAILED: ' + ', '.join(failures) or '모든 검사 통과'} "
      f"({len(failures)} 실패)")
sys.exit(1 if failures else 0)
