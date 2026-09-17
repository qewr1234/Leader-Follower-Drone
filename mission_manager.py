"""
mission_manager.py — Leader-Follower mission state machine

역할:
- 선두 출발/정지/착륙/소실 판단
- FOLLOW / HOVER / HOLD / LAND 상태 결정
- MARS-IMM controller 위에 올라가는 안전 상태 관리자
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
        """미션을 처음 상태로 되돌린다.

        조종사가 GUIDED로 넘기는 순간 main이 호출한다. 수동 비행 중 쌓인 소실
        타이머와 "추종한 적 있음" 기억을 지워, 인계 직후 LAND가 나가거나 확인 없이
        FOLLOW로 뛰어드는 일을 막는다.
        """
        self.state = S_WAIT_LEADER
        self.last_seen_t = None
        self.start_candidate_t = None
        self.landing_candidate_t = None
        # 한 번이라도 FOLLOW에 들어갔는가. 참이면 잠깐 놓쳤다 다시 찾았을 때
        # 출발 확인(0.7초 이동) 없이 바로 추종을 재개한다.
        self.has_followed = False

    def update(
        self,
        now,
        leader_visible,
        rel_est=None,
        rel_vel_est=None,
        leader_alt=None,
        leader_vel_world=None,
        pos_cov_trace=999.0,
    ):
        """
        입력:
        - leader_visible: vision/gps/esp/MARS-IMM 기준 선두 추적 가능 여부
        - rel_est: 후미 기준 선두 상대 위치 [front, right, up]
        - rel_vel_est: 상대 속도
        - leader_alt: 선두 절대(대지) 고도. None이면 착륙 판정을 하지 않는다 (C3)
        - leader_vel_world: ESP32/GPS에서 받은 선두 속도
        - pos_cov_trace: IMM 위치 공분산 trace
        """

        if leader_visible:
            self.last_seen_t = now
        else:
            # 가림 중에는 확인 타이머를 끊는다. 남겨 두면 가려진 시간이 타이머에
            # 합산돼 재획득 첫 프레임에 CONFIRMED_LANDING / FOLLOW가 날 수 있다.
            self.landing_candidate_t = None
            self.start_candidate_t = None
            if self.last_seen_t is None:
                # C1: 리더를 아직 한 번도 못 본 것은 "놓친" 것이 아니다.
                # 이 가드가 없으면 _lost_time이 999.0을 반환해 부팅 첫 프레임에
                # FAILSAFE_LAND로 직행한다 (이륙 직후 = 리더 획득 전이 정확히 이 조건).
                self._set_state(S_WAIT_LEADER, now)
                return self.state, self.command_policy()

            lost_time = self._lost_time(now)

            if lost_time < self.lost_hold_sec:
                self._set_state(S_LOST_HOLD, now)
            else:
                self._set_state(S_FAILSAFE_LAND, now)

            return self.state, self.command_policy()

        # 여기부터는 leader_visible=True
        hspeed, vz, z_for_landing = self._extract_motion(
            rel_vel_est=rel_vel_est,
            leader_alt=leader_alt,
            leader_vel_world=leader_vel_world,
        )

        # 추정 불확실성이 너무 크면 FOLLOW 금지
        if pos_cov_trace > 8.0:
            self.landing_candidate_t = None
            self.start_candidate_t = None
            self._set_state(S_LOST_HOLD, now)
            return self.state, self.command_policy()

        # 착륙 후보 판단
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
                self._set_state(S_CONFIRMED_LANDING, now)
                return self.state, self.command_policy()

            self._set_state(S_LANDING_CANDIDATE, now)
            return self.state, self.command_policy()

        else:
            self.landing_candidate_t = None

        # 출발 판단
        started_like = hspeed > self.start_speed_thresh

        if self.state in [S_WAIT_LEADER, S_READY_HOVER]:
            if started_like:
                if self.start_candidate_t is None:
                    self.start_candidate_t = now

                if now - self.start_candidate_t >= self.start_confirm_sec:
                    self._set_state(S_FOLLOW, now)
                else:
                    self._set_state(S_READY_HOVER, now)
            else:
                self.start_candidate_t = None
                self._set_state(S_READY_HOVER, now)

            return self.state, self.command_policy()

        # FOLLOW 중 선두가 거의 정지하면 정위치 유지(LEADER_HOVER)로.
        # H2: LEADER_HOVER를 여기서 처리해야 한다. 위 출발판단 블록에 두면
        # 진입 다음 프레임에 READY_HOVER로 덮여 정확히 1프레임만 살아남는다.
        # 히스테리시스: 진입 0.18 m/s, 복귀 0.25 m/s. 두 상태 모두 allow_follow=True라
        # "따라잡아서 상대속도가 준 것"과 "리더가 멈춘 것"을 혼동해도 제어는 끊기지 않는다.
        if self.state in [S_FOLLOW, S_LEADER_HOVER]:
            if self.state == S_FOLLOW and hspeed < self.hover_speed_thresh:
                self._set_state(S_LEADER_HOVER, now)
            elif self.state == S_LEADER_HOVER and hspeed > self.start_speed_thresh:
                self._set_state(S_FOLLOW, now)

            return self.state, self.command_policy()

        # 잠깐 놓쳤다가(LOST_HOLD) 또는 착륙 후보가 풀려서(LANDING_CANDIDATE) 여기 왔고,
        # 이미 추종하던 리더라면 바로 재개한다. 이전에는 READY_HOVER로 떨어져 리더가
        # 0.25 m/s 넘게 움직여야만 FOLLOW로 돌아왔다 — hspeed는 상대속도라 둘 다 호버
        # 중이면 0이고, 그러면 yaw 제어도 멈춘 채 리더가 천천히 시야를 벗어나는 걸
        # 지켜보다 소실 착륙으로 갔다. FAILSAFE_LAND / CONFIRMED_LANDING 에서 온 경우는
        # 그대로 READY_HOVER — 착륙 명령이 나간 뒤에는 출발 확인을 다시 거치는 편이 안전하다.
        if self.has_followed and self.state in (S_LOST_HOLD, S_LANDING_CANDIDATE):
            if hspeed > self.start_speed_thresh:
                self._set_state(S_FOLLOW, now)
            else:
                self._set_state(S_LEADER_HOVER, now)
            return self.state, self.command_policy()

        # 나머지는 안전하게 호버링
        self._set_state(S_READY_HOVER, now)
        return self.state, self.command_policy()

    def command_policy(self):
        """
        상태별 제어 정책.
        실제 velocity command는 controller에서 만들고,
        여기서는 controller를 쓸지/HOLD할지/LAND할지만 결정.
        """
        if self.state == S_WAIT_LEADER:
            return {"mode": "HOVER", "allow_follow": False, "land": False}

        if self.state == S_READY_HOVER:
            return {"mode": "HOVER", "allow_follow": False, "land": False}

        if self.state == S_FOLLOW:
            return {"mode": "FOLLOW", "allow_follow": True, "land": False}

        if self.state == S_LEADER_HOVER:
            return {"mode": "HOVER_TRACK", "allow_follow": True, "land": False}

        if self.state == S_LANDING_CANDIDATE:
            return {"mode": "HOVER_CONFIRM_LANDING", "allow_follow": False, "land": False}

        if self.state == S_CONFIRMED_LANDING:
            return {"mode": "LAND", "allow_follow": False, "land": True}

        if self.state == S_LOST_HOLD:
            return {"mode": "HOLD", "allow_follow": False, "land": False}

        if self.state == S_FAILSAFE_LAND:
            return {"mode": "FAILSAFE_LAND", "allow_follow": False, "land": True}

        return {"mode": "HOLD", "allow_follow": False, "land": False}

    def _extract_motion(self, rel_vel_est, leader_alt, leader_vel_world):
        hspeed = 0.0
        vz = 0.0

        if leader_vel_world is not None:
            v = np.asarray(leader_vel_world, dtype=float)
            if v.size >= 3:
                hspeed = float(np.linalg.norm(v[:2]))
                vz = float(v[2])
        elif rel_vel_est is not None:
            rv = np.asarray(rel_vel_est, dtype=float)
            if rv.size >= 3:
                hspeed = float(np.linalg.norm(rv[:2]))
                vz = float(rv[2])

        # C3: 절대(대지) 고도가 없으면 착륙 판정을 하지 않는다.
        # 상대 z로 대체하면 동고도 편대비행에서 값이 0 근처라 고도 조건이
        # 항상 만족되어 공중에서 CONFIRMED_LANDING이 난다.
        z_for_landing = float(leader_alt) if leader_alt is not None else None

        return hspeed, vz, z_for_landing

    def _lost_time(self, now):
        # last_seen_t is None은 update()에서 이미 걸러진다 (C1 가드).
        if self.last_seen_t is None:
            return 0.0
        return float(now - self.last_seen_t)

    def _set_state(self, new_state, now):
        self.state = new_state
        if new_state == S_FOLLOW:
            self.has_followed = True
