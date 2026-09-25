// test_imm_ekf.cpp — C++ 단독 불변식 검사 (gtest). 파이썬 오라클 대비 수치 일치는 test_fixes.py 'C++ 코어:' 와
// test_closed_loop.py --core cpp --compare 가 맡는다.
#include <cmath>

#include <gtest/gtest.h>

#include "mars_core/imm_ekf.hpp"

using namespace mars;

namespace {
Vec<3> v3(double a, double b, double c) { Vec<3> v; v.a[0] = a; v.a[1] = b; v.a[2] = c; return v; }
Mat<3, 3> diag3(double a, double b, double c) { Mat<3, 3> m; m(0, 0) = a; m(1, 1) = b; m(2, 2) = c; return m; }
}  // namespace

TEST(Mat, InverseAndDeterminant) {
    Mat<3, 3> m = diag3(2.0, 4.0, 8.0);
    m(0, 1) = 1.0;
    Mat<3, 3> inv;
    ASSERT_TRUE(inverse(m, inv));
    const Mat<3, 3> I = m * inv;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) EXPECT_NEAR(I(i, j), i == j ? 1.0 : 0.0, 1e-12);
    EXPECT_NEAR(determinant(m), 64.0, 1e-12);
    Mat<3, 3> singular;  // 0 행렬
    EXPECT_FALSE(inverse(singular, inv));
    EXPECT_DOUBLE_EQ(determinant(singular), 0.0);
}

TEST(ImmEkf, InitSetsStateAndFlags) {
    ImmEkf e;
    EXPECT_FALSE(e.initialized);
    e.init(v3(0.5, -0.2, 3.0));
    Vec<6> x; Mat<6, 6> P; e.get_state(x, P);
    EXPECT_DOUBLE_EQ(x.a[0], 0.5); EXPECT_DOUBLE_EQ(x.a[2], 3.0); EXPECT_DOUBLE_EQ(x.a[5], 0.0);
    EXPECT_DOUBLE_EQ(P(0, 0), 1.0); EXPECT_DOUBLE_EQ(P(3, 3), 2.0);
    EXPECT_TRUE(e.has_range_fix()); EXPECT_TRUE(e.has_vision_range_fix()); EXPECT_TRUE(e.is_reliable());
    ImmEkf g; g.init(v3(0, 0, 8), /*source_rgbd=*/false);
    EXPECT_TRUE(g.has_range_fix()); EXPECT_FALSE(g.has_vision_range_fix());
}

TEST(ImmEkf, PredictGrowsCovarianceAndCoast) {
    ImmEkf e; e.init(v3(0, 0, 3));
    Vec<6> x; Mat<6, 6> P0, P1; e.get_state(x, P0);
    for (int i = 0; i < 30; ++i) e.predict(1.0 / 30);
    e.get_state(x, P1);
    EXPECT_GT(P1.trace(), P0.trace());
    EXPECT_NEAR(e.range_coast_time, 1.0, 1e-9);
    EXPECT_TRUE(e.has_range_fix());
    for (int i = 0; i < 40; ++i) e.predict(1.0 / 30);
    EXPECT_FALSE(e.has_range_fix());   // 2.33 s > 2.0 s
    EXPECT_TRUE(e.is_finite());
}

TEST(ImmEkf, PositionUpdateShrinksCovarianceAndMovesState) {
    ImmEkf e; e.init(v3(0, 0, 3));
    e.predict(0.1);
    Vec<6> x; Mat<6, 6> Pb, Pa; e.get_state(x, Pb);
    const Mat<3, 3> R = diag3(0.0225, 0.0225, 0.0625);
    e.update_position3d(v3(0.1, 0.0, 3.2), &R);
    e.get_state(x, Pa);
    EXPECT_LT(Pa.trace(), Pb.trace());
    EXPECT_GT(x.a[2], 3.0); EXPECT_LT(x.a[2], 3.2);
    EXPECT_NEAR(e.mu[0] + e.mu[1], 1.0, 1e-12);
    EXPECT_DOUBLE_EQ(e.range_coast_time, 0.0);
}

TEST(ImmEkf, BearingUpdateDoesNotResetRangeCoast) {
    ImmEkf e; e.init(v3(0, 0, 3));
    e.predict(0.5);
    Vec<2> z; z.a[0] = 0.05; z.a[1] = 0.0;
    Mat<2, 2> R; R(0, 0) = 0.0009; R(1, 1) = 0.0009;
    e.update_bearing2d(z, R);
    EXPECT_DOUBLE_EQ(e.coast_time, 0.0);
    EXPECT_NEAR(e.range_coast_time, 0.5, 1e-12);
    Vec<6> x; Mat<6, 6> P; e.get_state(x, P);
    EXPECT_GT(x.a[0], 0.0);   // 방위가 +x 라 x 가 오른쪽으로
}

TEST(ImmEkf, EgoYawRotationIsExactForPureYaw) {
    ImmEkf e; e.init(v3(0, 0, 3));
    e.compensate_ego_yaw(M_PI / 2);     // 우회전 90° → 정면 표적이 카메라 −x
    Vec<6> x; Mat<6, 6> P; e.get_state(x, P);
    EXPECT_NEAR(x.a[0], -3.0, 1e-12); EXPECT_NEAR(x.a[2], 0.0, 1e-12);
    e.compensate_ego_yaw(-M_PI / 2);
    e.get_state(x, P);
    EXPECT_NEAR(x.a[0], 0.0, 1e-12); EXPECT_NEAR(x.a[2], 3.0, 1e-12);
    const double tr0 = 3 * 1.0 + 3 * 2.0;
    EXPECT_NEAR(P.trace(), tr0, 1e-12);  // 회전은 공분산 trace 를 보존
}

TEST(ImmEkf, InnovationUsesFusedState) {
    ImmEkf e; e.init(v3(0, 0, 3));
    Vec<3> y; Mat<3, 3> S;
    e.innovation_position3d(v3(0.2, 0, 3.5), nullptr, y, S);
    EXPECT_NEAR(y.a[0], 0.2, 1e-12); EXPECT_NEAR(y.a[2], 0.5, 1e-12);
    EXPECT_NEAR(S(2, 2), 1.0 + 0.0625, 1e-12);
    ImmEkf u; u.innovation_position3d(v3(0, 0, 3), nullptr, y, S);
    EXPECT_DOUBLE_EQ(S(0, 0), 999.0);
}

TEST(ImmEkf, VelocityHintAndReset) {
    ImmEkf e; e.init(v3(0, 0, 3));
    EXPECT_TRUE(e.apply_velocity_hint(v3(0, 0, 1.0), 0.1, 0.98));
    Vec<6> x; Mat<6, 6> P; e.get_state(x, P);
    EXPECT_NEAR(x.a[5], 0.1, 1e-12);
    EXPECT_NEAR(P(5, 5), 2.0 * 0.98, 1e-12);
    e.filters[0].x.a[0] = std::nan("");
    e.mark_dirty();
    EXPECT_FALSE(e.is_finite());
    e.reset();
    EXPECT_FALSE(e.initialized);
    EXPECT_FALSE(e.has_range_fix());
}

TEST(ImmEkf, LongCoastStaysFinite) {
    ImmEkf e; e.init(v3(0, 0, 3));
    for (int i = 0; i < 20000; ++i) e.predict(1.0 / 30);
    EXPECT_TRUE(e.is_finite());
    EXPECT_NEAR(e.mu[0] + e.mu[1], 1.0, 1e-9);
}
