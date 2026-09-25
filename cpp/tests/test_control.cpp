// test_control.cpp — 제어 법칙 C++ 단독 불변식 (gtest). 파이썬 오라클 대비 수치 일치는 test_fixes.py 'C++ 코어:' 가 맡는다.
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mars_core/control.hpp"

using namespace mars;

namespace {
const double NaN = std::numeric_limits<double>::quiet_NaN();
const double Inf = std::numeric_limits<double>::infinity();
Vec<3> v3(double a, double b, double c) { Vec<3> v; v.a[0] = a; v.a[1] = b; v.a[2] = c; return v; }
Vec<4> v4(double a, double b, double c, double d) { Vec<4> v; v.a[0] = a; v.a[1] = b; v.a[2] = c; v.a[3] = d; return v; }
bool all_zero(const Vec<4>& v) { for (double x : v.a) if (x != 0.0) return false; return true; }
const ControlGains G{};
}  // namespace

TEST(Clamp, NanInfGoToZeroThenClip) {
    EXPECT_DOUBLE_EQ(clamp(NaN, -0.35, 0.35), 0.0);
    EXPECT_DOUBLE_EQ(clamp(Inf, -0.35, 0.35), 0.0);
    EXPECT_DOUBLE_EQ(clamp(-Inf, -0.35, 0.35), 0.0);
    EXPECT_DOUBLE_EQ(clamp(NaN, 0.2, 0.5), 0.2);   // 0 을 같은 범위로 자른 값
    EXPECT_DOUBLE_EQ(clamp(9.0, -0.35, 0.35), 0.35);
    EXPECT_DOUBLE_EQ(clamp(-9.0, -0.35, 0.35), -0.35);
    EXPECT_DOUBLE_EQ(clamp(0.1, -0.35, 0.35), 0.1);
}

TEST(VelocityCmd, ProportionalAtSlot) {
    // 목표 거리에서 오차 0 → 명령 0 (yaw 도 0)
    const Vec<4> c = compute_velocity_cmd(G, v3(3.0, 0.0, 0.0), v3(0.0, 0.0, 0.0), 1.0, 3.0, std::nullopt, std::nullopt);
    EXPECT_TRUE(all_zero(c));
    // 1 m 멀면 KP_FORWARD·1 전진, 오른쪽 0.5 m 면 KP_RIGHT·0.5 우측 + yaw 우회전(+)
    const Vec<4> d = compute_velocity_cmd(G, v3(4.0, 0.5, -0.2), v3(0.0, 0.0, 0.0), 1.0, 3.0, std::nullopt, std::nullopt);
    EXPECT_NEAR(d.a[0], 0.22, 1e-12);
    EXPECT_NEAR(d.a[1], 0.28 * 0.5, 1e-12);
    EXPECT_NEAR(d.a[2], -(0.18 * -0.2), 1e-12);   // BODY_NED z = -up
    EXPECT_NEAR(d.a[3], 0.8 * std::atan2(0.5, 4.0), 1e-12);
}

TEST(VelocityCmd, Saturation) {
    const Vec<4> c = compute_velocity_cmd(G, v3(30.0, 30.0, 30.0), v3(5.0, 5.0, 5.0), 1.0, 3.0, std::nullopt, std::nullopt);
    EXPECT_DOUBLE_EQ(c.a[0], G.max_vx);
    EXPECT_DOUBLE_EQ(c.a[1], G.max_vy);
    EXPECT_DOUBLE_EQ(c.a[2], -G.max_vz);
    EXPECT_DOUBLE_EQ(c.a[3], G.max_yaw_rate);
    const Vec<4> d = compute_velocity_cmd(G, v3(0.5, -30.0, -30.0), v3(-5.0, -5.0, -5.0), 1.0, 3.0, std::nullopt, std::nullopt);
    EXPECT_DOUBLE_EQ(d.a[0], -G.max_vx);
    EXPECT_DOUBLE_EQ(d.a[1], -G.max_vy);
    EXPECT_DOUBLE_EQ(d.a[2], G.max_vz);
    EXPECT_DOUBLE_EQ(d.a[3], -G.max_yaw_rate);
}

TEST(VelocityCmd, NonFiniteInputsStop) {
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(NaN, 0.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, std::nullopt)));
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(4.0, 0.0, 0.0), v3(Inf, 0, 0), 1.0, 3.0, std::nullopt, std::nullopt)));
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(4.0, 0.0, 0.0), v3(0, 0, 0), 1.0, 3.0, v3(NaN, 0, 0), std::nullopt)));
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(4.0, 0.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, v3(0, -Inf, 0))));
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(4.0, 0.0, 0.0), v3(0, 0, 0), 1.0, NaN, std::nullopt, std::nullopt)));
    // NaN 공분산 trace 는 비교가 모두 거짓 → scale 1.0 (파이썬과 동일), 명령은 유한
    const Vec<4> c = compute_velocity_cmd(G, v3(4.0, 0.0, 0.0), v3(0, 0, 0), NaN, 3.0, std::nullopt, std::nullopt);
    EXPECT_TRUE(c.finite());
    EXPECT_NEAR(c.a[0], 0.22, 1e-12);
}

TEST(VelocityCmd, BehindCameraStops) {
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(0.0, 1.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, std::nullopt)));
    EXPECT_TRUE(all_zero(compute_velocity_cmd(G, v3(-2.0, 1.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, std::nullopt)));
}

TEST(VelocityCmd, UncertaintySlowdownBands) {
    const Vec<3> rel = v3(4.0, 0.0, 0.0), z = v3(0, 0, 0);
    EXPECT_NEAR(compute_velocity_cmd(G, rel, z, 1.0, 3.0, std::nullopt, std::nullopt).a[0], 0.22 * 1.0, 1e-12);
    EXPECT_NEAR(compute_velocity_cmd(G, rel, z, 2.0, 3.0, std::nullopt, std::nullopt).a[0], 0.22 * 1.0, 1e-12);   // 경계: > 2.0 이어야 감속
    EXPECT_NEAR(compute_velocity_cmd(G, rel, z, 3.0, 3.0, std::nullopt, std::nullopt).a[0], 0.22 * 0.75, 1e-12);
    EXPECT_NEAR(compute_velocity_cmd(G, rel, z, 4.0, 3.0, std::nullopt, std::nullopt).a[0], 0.22 * 0.75, 1e-12);
    EXPECT_NEAR(compute_velocity_cmd(G, rel, z, 4.1, 3.0, std::nullopt, std::nullopt).a[0], 0.22 * 0.55, 1e-12);
}

TEST(VelocityCmd, SlotErrorReplacesPositionErrorButNotYaw) {
    // 슬롯 오차 0 이면 위치 명령 0, yaw 는 여전히 리더 방위
    const Vec<4> c = compute_velocity_cmd(G, v3(4.0, 1.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, v3(0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(c.a[0], 0.0);
    EXPECT_DOUBLE_EQ(c.a[1], 0.0);
    EXPECT_DOUBLE_EQ(c.a[2], 0.0);
    EXPECT_NEAR(c.a[3], 0.8 * std::atan2(1.0, 4.0), 1e-12);   // 0.196 < MAX_YAW_RATE
    // 방위 14° 이상은 yaw 한계(0.35 rad/s)에 걸린다
    EXPECT_DOUBLE_EQ(compute_velocity_cmd(G, v3(4.0, 2.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, v3(0.0, 0.0, 0.0)).a[3], G.max_yaw_rate);
}

TEST(VelocityCmd, FeedforwardAddsKff) {
    const Vec<4> a = compute_velocity_cmd(G, v3(3.0, 0.0, 0.0), v3(0, 0, 0), 1.0, 3.0, std::nullopt, std::nullopt);
    const Vec<4> b = compute_velocity_cmd(G, v3(3.0, 0.0, 0.0), v3(0, 0, 0), 1.0, 3.0, v3(0.3, 0.1, 0.05), std::nullopt);
    EXPECT_NEAR(b.a[0] - a.a[0], 0.8 * 0.3, 1e-12);
    EXPECT_NEAR(b.a[1] - a.a[1], 0.8 * 0.1, 1e-12);
    EXPECT_NEAR(b.a[2] - a.a[2], -(0.8 * 0.05), 1e-12);
}

TEST(Smooth, AlphaAndDtEquivalence) {
    const Vec<4> zero{}, one = v4(0.1, 0.1, 0.1, 0.1);   // 한계(0.35/0.22/0.12) 아래
    const Vec<4> s0 = smooth_velocity_cmd(G, zero, one, 0.28, std::nullopt);
    EXPECT_NEAR(s0.a[0], 0.028, 1e-12);
    const Vec<4> s1 = smooth_velocity_cmd(G, zero, one, 0.28, G.smooth_ref_dt);
    EXPECT_NEAR(s1.a[0], 0.028, 1e-12);
    const Vec<4> s2 = smooth_velocity_cmd(G, zero, one, 0.28, 2.0 * G.smooth_ref_dt);
    EXPECT_NEAR(s2.a[0], 0.1 * (1.0 - (1.0 - 0.28) * (1.0 - 0.28)), 1e-12);   // 두 프레임 = 한 번의 2·dt
    // 출력은 축별 한계 안
    const Vec<4> s3 = smooth_velocity_cmd(G, v4(9, 9, 9, 9), v4(9, 9, 9, 9), 0.28, std::nullopt);
    EXPECT_DOUBLE_EQ(s3.a[0], G.max_vx); EXPECT_DOUBLE_EQ(s3.a[1], G.max_vy);
    EXPECT_DOUBLE_EQ(s3.a[2], G.max_vz); EXPECT_DOUBLE_EQ(s3.a[3], G.max_yaw_rate);
    // dt ≤ 0 은 alpha 그대로
    EXPECT_NEAR(smooth_velocity_cmd(G, zero, one, 0.28, 0.0).a[0], 0.028, 1e-12);
}

TEST(LeaderFF, SoftDeadbandAndLowPass) {
    const Vec<3> zero{};
    const double db = G.ff_deadband;
    EXPECT_DOUBLE_EQ(leader_velocity_ff(G, zero, v3(0.5 * db, 0, 0), 100.0).a[0], 0.0);             // 데드존 안 → 0
    EXPECT_NEAR(leader_velocity_ff(G, zero, v3(3.0 * db, 0, 0), 100.0).a[0], 2.0 * db, 1e-12);       // 기울기 1
    EXPECT_NEAR(leader_velocity_ff(G, zero, v3(0.3, 0, 0), G.ff_tau).a[0], (0.3 - db) * (1.0 - std::exp(-1.0)), 1e-12);
    EXPECT_NEAR(leader_velocity_ff(G, v3(0.3, 0, 0), std::nullopt, 100.0).a[0], 0.0, 1e-12);       // 입력 없음 → 0 으로 감쇠
    EXPECT_DOUBLE_EQ(leader_velocity_ff(G, zero, v3(Inf, 0, 0), 100.0).a[0], 0.0);                  // inf → 없는 것으로
    EXPECT_DOUBLE_EQ(leader_velocity_ff(G, zero, v3(25.0, 0, 0), 100.0).a[0], 0.0);                 // 20 m/s 상한 밖
    EXPECT_DOUBLE_EQ(leader_velocity_ff(G, v3(NaN, 0, 0), std::nullopt, 0.1).a[0], 0.0);            // 오염된 prev 는 0 에서 다시
    EXPECT_TRUE(leader_velocity_ff(G, zero, v3(0.3, 0, 0), -1.0).finite());                          // 음수 dt 는 0 으로
}

TEST(SelfLpf, StartsAtValueAndIgnoresNan) {
    const Vec<3> v = v3(0.3, 0, 0);
    EXPECT_DOUBLE_EQ(self_velocity_lpf(G, std::nullopt, v, 0.1).a[0], 0.3);
    EXPECT_NEAR(self_velocity_lpf(G, Vec<3>{}, v, G.ff_self_tau).a[0], 0.3 * (1.0 - std::exp(-1.0)), 1e-12);
    EXPECT_DOUBLE_EQ(self_velocity_lpf(G, v3(0.2, 0, 0), v3(NaN, 0, 0), 0.1).a[0], 0.2);   // NaN 입력 → 직전 값
    EXPECT_DOUBLE_EQ(self_velocity_lpf(G, std::nullopt, v3(NaN, 0, 0), 0.1).a[0], 0.0);
    EXPECT_DOUBLE_EQ(self_velocity_lpf(G, v3(NaN, 0, 0), v, 0.1).a[0], 0.3);                // 오염된 prev → 현재 값
}

TEST(Level, PitchMovesUpComponentAndKeepsNorm) {
    const double pitch = -10.0 * M_PI / 180.0;
    const Vec<3> lv = level_fru_by_roll_pitch(v3(3.0, 0.0, 0.0), 0.0, pitch);
    EXPECT_NEAR(lv.a[0], 3.0 * std::cos(pitch), 1e-12);
    EXPECT_NEAR(lv.a[1], 0.0, 1e-12);
    EXPECT_NEAR(lv.a[2], 3.0 * std::sin(pitch), 1e-12);
    EXPECT_NEAR(std::sqrt(dot(lv, lv)), 3.0, 1e-12);
    // roll 은 right/up 을 섞는다; 수평이면 항등
    const Vec<3> id = level_fru_by_roll_pitch(v3(1.0, 2.0, 3.0), 0.0, 0.0);
    EXPECT_NEAR(id.a[0], 1.0, 1e-12); EXPECT_NEAR(id.a[1], 2.0, 1e-12); EXPECT_NEAR(id.a[2], 3.0, 1e-12);
}

TEST(Sanitize, NonFiniteToZeroFalse) {
    Vec<4> out;
    EXPECT_TRUE(sanitize_cmd(v4(0.1, -0.2, 0.0, 0.3), out));
    EXPECT_DOUBLE_EQ(out.a[1], -0.2);
    EXPECT_FALSE(sanitize_cmd(v4(0.1, NaN, 0.0, 0.3), out));
    EXPECT_TRUE(all_zero(out));
    EXPECT_FALSE(sanitize_cmd(v4(Inf, 0.0, 0.0, 0.0), out));
    EXPECT_TRUE(all_zero(out));
}
