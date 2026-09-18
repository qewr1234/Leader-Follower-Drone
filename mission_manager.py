"""
mission_manager.py — Leader-Follower mission state machine

선두의 출발/정지/착륙/소실을 판단해 FOLLOW / HOVER / HOLD / LAND 정책을 낸다.
MARS-IMM controller 위에 올라가는 안전 상태 관리자.
"""

import numpy as np


S_WAIT_LEADER = "WAIT_LEADER"
S_READY_HOVER = "READY_HOVER"
S_FOLLOW = "FOLLOW"
S_LEADER_HOVER = "LEADER_HOVER"
S_LANDING_CANDIDATE = "LANDING_CANDIDATE"
S_CONFIRMED_LANDING = "CONFIRMED_LANDING"
S_LOST_HOLD = "LOST_HOLD"
S_FAILSAFE_LAND = "FAILSAFE_LAND"

# 상태별 제어 정책. 실제 velocity command 는 main 이 만들고, 여기서는 따라갈지/HOLD/LAND 만 정한다.
_POLICY = {
    S_WAIT_LEADER:       {"mode": "HOVER",                 "allow_follow": False, "land": False},
    S_READY_HOVER:       {"mode": "HOVER",                 "allow_follow": False, "land": False},
    S_FOLLOW:            {"mode": "FOLLOW",                "allow_follow": True,  "land": False},
    S_LEADER_HOVER:      {"mode": "HOVER_TRACK",           "allow_follow": True,  "land": False},
    S_LANDING_CANDIDATE: {"mode": "HOVER_CONFIRM_LANDING", "allow_follow": False, "land": False},
    S_CONFIRMED_LANDING: {"mode": "LAND",                  "allow_follow": False, "land": True},
    S_LOST_HOLD:         {"mode": "HOLD",                  "allow_follow": False, "land": False},
    S_FAILSAFE_LAND:     {"mode": "FAILSAFE_LAND",         "allow_follow": False, "land": True},
}
_POLICY_UNKNOWN = {"mode": "HOLD", "allow_follow": False, "land": False}


class MissionManager:
    def __init__(
        self,
        start_speed_thresh=0.25,
        start_confirm_sec=0.7,
        hover_speed_thresh=0.18,
        landing_z_thresh=0.65,
        landing_vz_thresh=-0.10,
        landing_hspeed_thresh=0.25,
        landing_confirm_sec=1.8,
        lost_hold_sec=8.0,   # + imm.range_coast_max_sec(2.0) = 소실 후 총 10초에 착륙
    ):
        self.start_speed_thresh = float(start_speed_thresh)
        self.start_confirm_sec = float(start_confirm_sec)
        self.hover_speed_thresh = float(hover_speed_thresh)
        self.landing_z_thresh = float(landing_z_thresh)
        self.landing_vz_thresh = float(landing_vz_thresh)
        self.landing_hspeed_thresh = float(landing_hspeed_thresh)
        self.landing_confirm_sec = float(landing_confirm_sec)
        self.lost_hold_sec = float(lost_hold_sec)
        self.reset()

    def reset(self):
        """미션을 처음 상태로. 조종사가 GUIDED로 넘기는 순간 main이 호출한다 — 수동 비행 중 쌓인
        소실 타이머와 '추종한 적 있음' 기억을 지워, 인계 직후 LAND 가 나가거나 확인 없이 FOLLOW 로
        뛰어드는 일을 막는다."""
        self.state = S_WAIT_LEADER
        self.last_seen_t = None
        self.start_candidate_t = None
        self.landing_candidate_t = None
        # 한 번이라도 FOLLOW 에 들어갔는가. 참이면 잠깐 놓쳤다 다시 찾았을 때 출발 확인 없이 재개.
        self.has_followed = False

    def update(self, now, leader_visible, rel_est=None, rel_vel_est=None,
               leader_alt=None, leader_vel_world=None, pos_cov_trace=999.0):
        """
        - leader_visible: 거리를 아는가 (RGB-D 또는 ESP32) — main 이 ekf.has_range_fix() 로 준다
        - rel_est / rel_vel_est: 후미 기준 선두 상대 위치·속도 [front, right, up]
        - leader_alt: 선두 절대(대지) 고도. None 이면 착륙 판정을 하지 않는다 (C3)
        - leader_vel_world: ESP32/GPS 선두 절대 속도 (있으면 상대속도 대신 이걸로 출발/착륙 판단)
        - pos_cov_trace: IMM 위치 공분산 trace
        """
        if not leader_visible:
            # 가림 중에는 확인 타이머를 끊는다. 남겨 두면 가려진 시간이 타이머에 합산돼
            # 재획득 첫 프레임에 CONFIRMED_LANDING / FOLLOW 가 날 수 있다.
            self._clear_timers()
            if self.last_seen_t is None:
                # C1: 아직 한 번도 못 본 것은 '놓친' 것이 아니다 (이륙 직후가 정확히 이 조건).
                return self._go(S_WAIT_LEADER)
            lost = now - self.last_seen_t
            return self._go(S_LOST_HOLD if lost < self.lost_hold_sec else S_FAILSAFE_LAND)

        self.last_seen_t = now
        hspeed, vz, z_for_landing = self._extract_motion(rel_vel_est, leader_alt, leader_vel_world)

        # 추정 불확실성이 너무 크면 FOLLOW 금지
        if pos_cov_trace > 8.0:
            self._clear_timers()
            return self._go(S_LOST_HOLD)

        # 착륙 후보: 지면 근처 + 하강 중 + 거의 정지
        landing_like = (
            z_for_landing is not None
            and z_for_landing < self.landing_z_thresh
            and vz < self.landing_vz_thresh
            and hspeed < self.landing_hspeed_thresh
        )
        if landing_like:
            if self.landing_candidate_t is None:
                self.landing_candidate_t = now
            if now - self.landing_candidate_t >= self.landing_confirm_sec:
                return self._go(S_CONFIRMED_LANDING)
            return self._go(S_LANDING_CANDIDATE)
        self.landing_candidate_t = None

        started_like = hspeed > self.start_speed_thresh

        # 출발 확인: start_confirm_sec 동안 움직여야 FOLLOW
        if self.state in (S_WAIT_LEADER, S_READY_HOVER):
            if not started_like:
                self.start_candidate_t = None
                return self._go(S_READY_HOVER)
            if self.start_candidate_t is None:
                self.start_candidate_t = now
            return self._go(S_FOLLOW if now - self.start_candidate_t >= self.start_confirm_sec else S_READY_HOVER)

        # FOLLOW ↔ LEADER_HOVER 히스테리시스 (진입 0.18, 복귀 0.25 m/s). 둘 다 allow_follow 라
        # '따라잡아서 상대속도가 준 것'과 '리더가 멈춘 것'을 혼동해도 제어는 끊기지 않는다.
        if self.state in (S_FOLLOW, S_LEADER_HOVER):
            if self.state == S_FOLLOW and hspeed < self.hover_speed_thresh:
                return self._go(S_LEADER_HOVER)
            if self.state == S_LEADER_HOVER and started_like:
                return self._go(S_FOLLOW)
            return self._go(self.state)

        # 잠깐 놓쳤다가(LOST_HOLD) 또는 착륙 후보가 풀려서(LANDING_CANDIDATE) 온 경우, 이미 추종하던
        # 리더라면 바로 재개한다 — READY_HOVER 로 떨어뜨리면 hspeed(상대속도)가 0 인 호버 리더에게는
        # 영원히 돌아가지 못하고, yaw 제어도 멈춘 채 시야 이탈 → 소실 착륙으로 간다.
        # FAILSAFE_LAND / CONFIRMED_LANDING 에서 온 경우는 READY_HOVER — 착륙 명령 뒤에는 출발 확인을 다시.
        if self.has_followed and self.state in (S_LOST_HOLD, S_LANDING_CANDIDATE):
            return self._go(S_FOLLOW if started_like else S_LEADER_HOVER)

        return self._go(S_READY_HOVER)

    def command_policy(self):
        return dict(_POLICY.get(self.state, _POLICY_UNKNOWN))

    def _extract_motion(self, rel_vel_est, leader_alt, leader_vel_world):
        """(수평 속도, 수직 속도(up +), 착륙 판정용 고도). 선두 절대 속도가 있으면 그것을, 없으면 상대속도."""
        v = leader_vel_world if leader_vel_world is not None else rel_vel_est
        hspeed = vz = 0.0
        if v is not None:
            v = np.asarray(v, dtype=float)
            if v.size >= 3:
                hspeed, vz = float(np.hypot(v[0], v[1])), float(v[2])
        # C3: 절대 고도가 없으면 착륙 판정을 하지 않는다. 상대 z 로 대체하면 동고도 편대에서 항상 0 근처라
        # 고도 조건이 늘 만족돼 공중에서 CONFIRMED_LANDING 이 난다.
        z_for_landing = float(leader_alt) if leader_alt is not None else None
        return hspeed, vz, z_for_landing

    def _clear_timers(self):
        self.landing_candidate_t = None
        self.start_candidate_t = None

    def _go(self, new_state):
        self.state = new_state
        if new_state == S_FOLLOW:
            self.has_followed = True
        return self.state, self.command_policy()
