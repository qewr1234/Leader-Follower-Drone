// test_mission.cpp — 미션 상태머신 C++ 단독 검사 (gtest). 파이썬 오라클과의 상태열 일치는 test_fixes.py 'C++ 코어:' 가 맡는다.
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mars_core/mission.hpp"

using namespace mars;
using S = MissionState;

namespace {
Vec<3> v3(double a, double b, double c) { Vec<3> v; v.a[0] = a; v.a[1] = b; v.a[2] = c; return v; }
const std::optional<Vec<3>> none;
const std::optional<Vec<3>> still = v3(0.0, 0.0, 0.0);
const std::optional<Vec<3>> moving = v3(0.4, 0.0, 0.0);

// 리더가 보이고 절대 속도 v 로 움직이는 한 프레임 (고도는 알 수 없음 → 착륙 판정 없음)
S step(MissionManager& m, double t, const std::optional<Vec<3>>& v, double cov = 1.0) {
    return m.update(t, true, v3(3.0, 0.0, 0.0), v3(0.0, 0.0, 0.0), std::nullopt, none, cov, v);
}

// 출발 확인을 지나 FOLLOW 까지 (t=0 … 1.0)
MissionManager followed() {
    MissionManager m;
    for (double t = 0.0; t <= 1.0; t += 0.1) step(m, t, moving);
    EXPECT_EQ(m.state, S::FOLLOW);
    EXPECT_TRUE(m.has_followed);
    return m;
}
}  // namespace

TEST(Mission, NamesRoundTrip) {
    for (int i = 0; i <= static_cast<int>(S::FAILSAFE_LAND); ++i) {
        const auto s = static_cast<S>(i);
        const auto back = mission_state_from_name(mission_state_name(s));
        ASSERT_TRUE(back.has_value());
        EXPECT_EQ(*back, s);
    }
    EXPECT_FALSE(mission_state_from_name("BOGUS").has_value());
}

TEST(Mission, PolicyTable) {
    EXPECT_EQ(mission_policy_of(S::FOLLOW).mode, "FOLLOW");
    EXPECT_TRUE(mission_policy_of(S::FOLLOW).allow_follow);
    EXPECT_TRUE(mission_policy_of(S::LEADER_HOVER).allow_follow);
    EXPECT_FALSE(mission_policy_of(S::LOST_HOLD).allow_follow);
    EXPECT_TRUE(mission_policy_of(S::CONFIRMED_LANDING).land);
    EXPECT_TRUE(mission_policy_of(S::FAILSAFE_LAND).land);
    for (int i = 0; i <= static_cast<int>(S::FAILSAFE_LAND); ++i) {
        const auto s = static_cast<S>(i);
        if (s != S::CONFIRMED_LANDING && s != S::FAILSAFE_LAND) EXPECT_FALSE(mission_policy_of(s).land) << mission_state_name(s);
    }
}

TEST(Mission, NeverSeenIsWaitNotLost) {   // C1
    MissionManager m;
    for (double t = 0.0; t < 30.0; t += 1.0) EXPECT_EQ(m.update(t, false, none, none, std::nullopt, none, 999.0, none), S::WAIT_LEADER);
    EXPECT_FALSE(m.command_policy().land);
}

TEST(Mission, StartConfirmThenFollow) {
    MissionManager m;
    EXPECT_EQ(step(m, 0.0, still), S::READY_HOVER);
    EXPECT_EQ(step(m, 0.1, moving), S::READY_HOVER);     // 후보 시작
    EXPECT_EQ(step(m, 0.7, moving), S::READY_HOVER);     // 0.6 s < 0.7
    EXPECT_EQ(step(m, 0.8, moving), S::FOLLOW);          // 0.7 s ≥ 0.7
    EXPECT_TRUE(m.has_followed);
    EXPECT_TRUE(m.command_policy().allow_follow);
}

TEST(Mission, StartTimerResetsWhenLeaderStops) {
    MissionManager m;
    step(m, 0.0, moving);
    step(m, 0.5, still);                                  // 후보 취소
    EXPECT_FALSE(m.start_candidate_t.has_value());
    EXPECT_EQ(step(m, 0.6, moving), S::READY_HOVER);
    EXPECT_EQ(step(m, 1.2, moving), S::READY_HOVER);     // 0.6 s 뿐
    EXPECT_EQ(step(m, 1.3, moving), S::FOLLOW);
}

TEST(Mission, FollowHoverHysteresis) {
    MissionManager m = followed();
    EXPECT_EQ(step(m, 2.0, v3(0.2, 0.0, 0.0)), S::FOLLOW);         // 0.18 ≤ 0.2 → 유지
    EXPECT_EQ(step(m, 2.1, v3(0.1, 0.0, 0.0)), S::LEADER_HOVER);   // < 0.18
    EXPECT_EQ(step(m, 2.2, v3(0.2, 0.0, 0.0)), S::LEADER_HOVER);   // 0.2 ≤ 0.25 → 유지
    EXPECT_EQ(step(m, 2.3, v3(0.3, 0.0, 0.0)), S::FOLLOW);         // > 0.25 즉시 (확인 없음)
    EXPECT_TRUE(m.command_policy().allow_follow);
}

TEST(Mission, LostAfterFollowHoldsThenFailsafe) {
    MissionManager m = followed();
    EXPECT_EQ(m.update(2.0, false, none, none, std::nullopt, none, 999.0, none), S::LOST_HOLD);
    EXPECT_EQ(m.update(1.0 + 7.99, false, none, none, std::nullopt, none, 999.0, none), S::LOST_HOLD);
    EXPECT_FALSE(m.command_policy().land);
    EXPECT_EQ(m.update(1.0 + 8.0, false, none, none, std::nullopt, none, 999.0, none), S::FAILSAFE_LAND);
    EXPECT_TRUE(m.command_policy().land);
    // 착륙 명령 뒤 재획득은 출발 확인부터
    EXPECT_EQ(step(m, 9.5, still), S::READY_HOVER);
}

TEST(Mission, ReacquireResumesWithoutStartConfirm) {
    MissionManager m = followed();
    m.update(2.0, false, none, none, std::nullopt, none, 999.0, none);
    EXPECT_EQ(m.state, S::LOST_HOLD);
    EXPECT_EQ(step(m, 3.0, still), S::LEADER_HOVER);   // 정지 리더도 바로 추적 재개
    EXPECT_TRUE(m.command_policy().allow_follow);
    m.update(4.0, false, none, none, std::nullopt, none, 999.0, none);
    EXPECT_EQ(step(m, 5.0, moving), S::FOLLOW);        // 움직이면 바로 FOLLOW
}

TEST(Mission, ReacquireBeforeAnyFollowNeedsConfirm) {
    MissionManager m;
    step(m, 0.0, still);                                   // READY_HOVER, 본 적 있음
    EXPECT_EQ(m.update(1.0, false, none, none, std::nullopt, none, 999.0, none), S::LOST_HOLD);
    EXPECT_EQ(step(m, 2.0, moving), S::READY_HOVER);      // has_followed 없음 → 확인부터
}

TEST(Mission, LandingCandidateThenConfirmed) {
    MissionManager m = followed();
    auto land_frame = [&](double t) { return m.update(t, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.0, 0.0, -0.3)); };
    EXPECT_EQ(land_frame(2.0), S::LANDING_CANDIDATE);
    EXPECT_FALSE(m.command_policy().allow_follow);
    EXPECT_EQ(land_frame(3.75), S::LANDING_CANDIDATE);     // 1.75 s < 1.8
    EXPECT_EQ(land_frame(4.0), S::CONFIRMED_LANDING);      // 2.0 s ≥ 1.8 (3.8−2.0 은 부동소수점으로 1.8 보다 작다)
    EXPECT_TRUE(m.command_policy().land);
}

TEST(Mission, LandingCandidateReleasedResumesTracking) {
    MissionManager m = followed();
    m.update(2.0, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.0, 0.0, -0.3));
    EXPECT_EQ(m.state, S::LANDING_CANDIDATE);
    EXPECT_EQ(step(m, 2.5, still), S::LEADER_HOVER);       // 하강 멈춤 → 추적 재개
    EXPECT_FALSE(m.landing_candidate_t.has_value());
}

TEST(Mission, NoAltitudeNoLanding) {   // C3
    MissionManager m = followed();
    for (double t = 2.0; t < 10.0; t += 0.1)
        EXPECT_NE(m.update(t, true, v3(3, 0, -2.0), v3(0, 0, -0.5), std::nullopt, none, 1.0, v3(0.0, 0.0, -0.5)), S::LANDING_CANDIDATE);
    EXPECT_FALSE(m.command_policy().land);
}

TEST(Mission, LandingNeedsAllThree) {
    MissionManager m = followed();
    EXPECT_NE(m.update(2.0, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.4, 0.0, -0.3)), S::LANDING_CANDIDATE);   // 수평 이동 중
    EXPECT_NE(m.update(2.1, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.0, 0.0, 0.0)), S::LANDING_CANDIDATE);    // 하강 아님
    EXPECT_NE(m.update(2.2, true, v3(3, 0, 0), v3(0, 0, 0), 2.0, none, 1.0, v3(0.0, 0.0, -0.3)), S::LANDING_CANDIDATE);   // 높음
}

TEST(Mission, OcclusionClearsTimers) {
    MissionManager m = followed();
    m.update(2.0, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.0, 0.0, -0.3));   // 후보 시작 t=2
    m.update(2.5, false, none, none, std::nullopt, none, 999.0, none);                     // 가림
    EXPECT_FALSE(m.landing_candidate_t.has_value());
    EXPECT_EQ(m.update(4.0, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(0.0, 0.0, -0.3)), S::LANDING_CANDIDATE);   // 2 s 지났어도 재시작
    EXPECT_DOUBLE_EQ(*m.landing_candidate_t, 4.0);
}

TEST(Mission, HighCovarianceHolds) {
    MissionManager m = followed();
    EXPECT_EQ(step(m, 2.0, moving, 8.5), S::LOST_HOLD);
    EXPECT_FALSE(m.command_policy().allow_follow);
    EXPECT_EQ(step(m, 2.1, moving, 1.0), S::FOLLOW);       // 회복 → 바로 재개
}

TEST(Mission, VelocitySourcePriority) {
    // world > body > rel. world 가 정지라면 body 가 움직여도 정지로 본다.
    MissionManager m;
    for (double t = 0.0; t <= 1.0; t += 0.1) m.update(t, true, v3(3, 0, 0), v3(0.4, 0, 0), std::nullopt, v3(0, 0, 0), 1.0, v3(0.4, 0, 0));
    EXPECT_EQ(m.state, S::READY_HOVER);
    MissionManager m2;
    for (double t = 0.0; t <= 1.0; t += 0.1) m2.update(t, true, v3(3, 0, 0), v3(0.0, 0, 0), std::nullopt, none, 1.0, v3(0.4, 0, 0));
    EXPECT_EQ(m2.state, S::FOLLOW);
    MissionManager m3;   // 상대 속도만 있을 때의 폴백
    for (double t = 0.0; t <= 1.0; t += 0.1) m3.update(t, true, v3(3, 0, 0), v3(0.4, 0, 0), std::nullopt, none, 1.0, none);
    EXPECT_EQ(m3.state, S::FOLLOW);
}

TEST(Mission, NanVelocityChangesNothing) {
    // NaN 속도는 모든 비교가 거짓: 출발도 정지도 착륙도 아니다 (파이썬 np.hypot/비교와 같은 의미) → 현재 상태 유지
    const double NaN = std::numeric_limits<double>::quiet_NaN();
    MissionManager m = followed();
    EXPECT_EQ(m.update(2.0, true, v3(3, 0, 0), v3(0, 0, 0), 0.5, none, 1.0, v3(NaN, 0.0, -0.3)), S::FOLLOW);
    EXPECT_FALSE(m.landing_candidate_t.has_value());
    MissionManager m2;
    EXPECT_EQ(m2.update(0.0, true, v3(3, 0, 0), v3(0, 0, 0), std::nullopt, none, 1.0, v3(NaN, 0.0, 0.0)), S::READY_HOVER);
    EXPECT_FALSE(m2.start_candidate_t.has_value());
}

TEST(Mission, ResetClearsEverything) {
    MissionManager m = followed();
    m.update(2.0, false, none, none, std::nullopt, none, 999.0, none);
    m.reset();
    EXPECT_EQ(m.state, S::WAIT_LEADER);
    EXPECT_FALSE(m.last_seen_t.has_value());
    EXPECT_FALSE(m.has_followed);
    EXPECT_EQ(m.update(3.0, false, none, none, std::nullopt, none, 999.0, none), S::WAIT_LEADER);   // 리셋 뒤 소실은 다시 '본 적 없음'
}
