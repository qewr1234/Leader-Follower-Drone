// mission.hpp — mission_manager.py 의 이식: 선두 출발/정지/착륙/소실 판단 → FOLLOW / HOVER / HOLD / LAND 정책.
#pragma once

#include <optional>
#include <string>

#include "mars_core/mat.hpp"

namespace mars {

enum class MissionState { WAIT_LEADER, READY_HOVER, FOLLOW, LEADER_HOVER, LANDING_CANDIDATE, CONFIRMED_LANDING, LOST_HOLD, FAILSAFE_LAND };

const char* mission_state_name(MissionState s);
std::optional<MissionState> mission_state_from_name(const std::string& name);

struct MissionPolicy {
    std::string mode;
    bool allow_follow = false;
    bool land = false;
};

MissionPolicy mission_policy_of(MissionState s);

struct MissionParams {
    double start_speed_thresh = 0.25;
    double start_confirm_sec = 0.7;
    double hover_speed_thresh = 0.18;
    double landing_z_thresh = 0.65;
    double landing_vz_thresh = -0.10;
    double landing_hspeed_thresh = 0.25;
    double landing_confirm_sec = 1.8;
    double lost_hold_sec = 8.0;
};

class MissionManager {
public:
    explicit MissionManager(const MissionParams& p = MissionParams{});
    void reset();
    // update(): 파이썬과 같은 인자·의미. 반환 = 새 상태 (정책은 command_policy()).
    MissionState update(double now, bool leader_visible, const std::optional<Vec<3>>& rel_est, const std::optional<Vec<3>>& rel_vel_est,
                        std::optional<double> leader_alt, const std::optional<Vec<3>>& leader_vel_world, double pos_cov_trace,
                        const std::optional<Vec<3>>& leader_vel_body);
    MissionPolicy command_policy() const { return mission_policy_of(state); }
    const MissionParams& params() const { return p_; }
    MissionParams& params_mut() { return p_; }   // 파이썬 속성(lost_hold_sec 등) 읽기·쓰기용

    MissionState state = MissionState::WAIT_LEADER;
    std::optional<double> last_seen_t;
    std::optional<double> start_candidate_t;
    std::optional<double> landing_candidate_t;
    bool has_followed = false;

private:
    MissionState go(MissionState s);
    void clear_timers();
    MissionParams p_;
};

}  // namespace mars
