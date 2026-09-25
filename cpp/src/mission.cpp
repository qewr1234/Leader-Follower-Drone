// mission.cpp — mission_manager.py 의 이식. 분기 순서를 그대로 지킨다.
#include "mars_core/mission.hpp"

#include <cmath>

namespace mars {

const char* mission_state_name(MissionState s) {
    switch (s) {
        case MissionState::WAIT_LEADER: return "WAIT_LEADER";
        case MissionState::READY_HOVER: return "READY_HOVER";
        case MissionState::FOLLOW: return "FOLLOW";
        case MissionState::LEADER_HOVER: return "LEADER_HOVER";
        case MissionState::LANDING_CANDIDATE: return "LANDING_CANDIDATE";
        case MissionState::CONFIRMED_LANDING: return "CONFIRMED_LANDING";
        case MissionState::LOST_HOLD: return "LOST_HOLD";
        case MissionState::FAILSAFE_LAND: return "FAILSAFE_LAND";
    }
    return "?";
}

std::optional<MissionState> mission_state_from_name(const std::string& n) {
    for (int i = 0; i <= static_cast<int>(MissionState::FAILSAFE_LAND); ++i) {
        const auto s = static_cast<MissionState>(i);
        if (n == mission_state_name(s)) return s;
    }
    return std::nullopt;
}

MissionPolicy mission_policy_of(MissionState s) {
    switch (s) {
        case MissionState::WAIT_LEADER: return {"HOVER", false, false};
        case MissionState::READY_HOVER: return {"HOVER", false, false};
        case MissionState::FOLLOW: return {"FOLLOW", true, false};
        case MissionState::LEADER_HOVER: return {"HOVER_TRACK", true, false};
        case MissionState::LANDING_CANDIDATE: return {"HOVER_CONFIRM_LANDING", false, false};
        case MissionState::CONFIRMED_LANDING: return {"LAND", false, true};
        case MissionState::LOST_HOLD: return {"HOLD", false, false};
        case MissionState::FAILSAFE_LAND: return {"FAILSAFE_LAND", false, true};
    }
    return {"HOLD", false, false};
}

MissionManager::MissionManager(const MissionParams& p) : p_(p) { reset(); }

void MissionManager::reset() {
    state = MissionState::WAIT_LEADER;
    last_seen_t.reset();
    start_candidate_t.reset();
    landing_candidate_t.reset();
    has_followed = false;
}

void MissionManager::clear_timers() {
    landing_candidate_t.reset();
    start_candidate_t.reset();
}

MissionState MissionManager::go(MissionState s) {
    state = s;
    if (s == MissionState::FOLLOW) has_followed = true;
    return state;
}

MissionState MissionManager::update(double now, bool leader_visible, const std::optional<Vec<3>>& /*rel_est*/, const std::optional<Vec<3>>& rel_vel_est,
                                    std::optional<double> leader_alt, const std::optional<Vec<3>>& leader_vel_world, double pos_cov_trace,
                                    const std::optional<Vec<3>>& leader_vel_body) {
    using S = MissionState;
    if (!leader_visible) {
        clear_timers();
        if (!last_seen_t) return go(S::WAIT_LEADER);            // C1: 한 번도 못 본 것은 놓친 것이 아니다
        const double lost = now - *last_seen_t;
        return go(lost < p_.lost_hold_sec ? S::LOST_HOLD : S::FAILSAFE_LAND);
    }

    last_seen_t = now;
    // _extract_motion: ESP32 절대(ENU) > 자기+상대(FRU) > 상대
    const std::optional<Vec<3>>& v = leader_vel_world ? leader_vel_world : (leader_vel_body ? leader_vel_body : rel_vel_est);
    double hspeed = 0.0, vz = 0.0;
    if (v) { hspeed = std::hypot(v->a[0], v->a[1]); vz = v->a[2]; }
    const std::optional<double> z_for_landing = leader_alt;

    if (pos_cov_trace > 8.0) {
        clear_timers();
        return go(S::LOST_HOLD);
    }

    const bool landing_like = z_for_landing && *z_for_landing < p_.landing_z_thresh && vz < p_.landing_vz_thresh && hspeed < p_.landing_hspeed_thresh;
    if (landing_like) {
        if (!landing_candidate_t) landing_candidate_t = now;
        if (now - *landing_candidate_t >= p_.landing_confirm_sec) return go(S::CONFIRMED_LANDING);
        return go(S::LANDING_CANDIDATE);
    }
    landing_candidate_t.reset();

    const bool started_like = hspeed > p_.start_speed_thresh;

    if (state == S::WAIT_LEADER || state == S::READY_HOVER) {
        if (!started_like) {
            start_candidate_t.reset();
            return go(S::READY_HOVER);
        }
        if (!start_candidate_t) start_candidate_t = now;
        return go(now - *start_candidate_t >= p_.start_confirm_sec ? S::FOLLOW : S::READY_HOVER);
    }

    if (state == S::FOLLOW || state == S::LEADER_HOVER) {
        if (state == S::FOLLOW && hspeed < p_.hover_speed_thresh) return go(S::LEADER_HOVER);
        if (state == S::LEADER_HOVER && started_like) return go(S::FOLLOW);
        return go(state);
    }

    if (has_followed && (state == S::LOST_HOLD || state == S::LANDING_CANDIDATE))
        return go(started_like ? S::FOLLOW : S::LEADER_HOVER);

    return go(S::READY_HOVER);
}

}  // namespace mars
