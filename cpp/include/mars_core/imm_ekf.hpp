// imm_ekf.hpp — IMM-EKF (CV + Coordinated-Turn), imm_ekf.py 의 1:1 이식.
// 상태 x = [px, py, pz, vx, vy, vz], 카메라 좌표계 (x=right, y=down, z=forward).
// 파이썬 오라클과 같은 연산 순서를 지킨다 — 차등 검사가 그것을 보장한다.
#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "mars_core/mat.hpp"

namespace mars {

struct ImmParams {
    double sigma_xy = 0.15;         // config imm.sigma_xy — 기본 R 의 x/y 표준편차
    double sigma_z = 0.25;          // config imm.sigma_z
    double max_coast_sec = 2.0;     // config imm.max_coast_sec
    double range_coast_max_sec = 2.0;
    double sigma_a_cv = 0.8;
    double sigma_a_ct = 1.2;
    double p0_pos = 1.0;
    double p0_vel = 2.0;
    std::array<std::array<double, 2>, 2> trans_prob{{{0.95, 0.05}, {0.10, 0.90}}};
    std::array<double, 2> mu0{{0.7, 0.3}};
};

class SingleEkf {
public:
    explicit SingleEkf(int model_id, const ImmParams& p);
    void reset(const Vec<6>& x, const Mat<6, 6>& P);
    void predict(double dt);
    double update_position3d(const Vec<3>& z, const Mat<3, 3>& R);
    double update_bearing2d(const Vec<2>& z, const Mat<2, 2>& R);

    Vec<6> x;
    Mat<6, 6> P;
    double omega = 0.0;
    std::optional<double> prev_heading;
    double dt_since_update = 0.0;
    int model_id;

private:
    template <std::size_t D>
    double update(const Vec<D>& z, const Vec<D>& h, const Mat<D, 6>& H, const Mat<D, D>& R);
    void estimate_omega();
    ImmParams p_;      // 값 복사 — 참조를 잡으면 ImmEkf 복사/재할당 시 매달린 참조가 된다
};

class ImmEkf {
public:
    static constexpr int N_MODELS = 2;

    explicit ImmEkf(const ImmParams& p = ImmParams{});
    ImmEkf(const ImmEkf&) = default;
    ImmEkf& operator=(const ImmEkf&) = default;

    void init(const Vec<3>& z, bool source_rgbd = true);
    void reset();
    void predict(double dt);
    void update_position3d(const Vec<3>& z, const Mat<3, 3>* R, bool source_rgbd = true);
    void update_bearing2d(const Vec<2>& z, const Mat<2, 2>& R);
    void innovation_position3d(const Vec<3>& z, const Mat<3, 3>* R, Vec<3>& y, Mat<3, 3>& S) const;
    void innovation_bearing2d(const Vec<2>& z, const Mat<2, 2>& R, Vec<2>& y, Mat<2, 2>& S) const;
    void on_lost(double dt) { coast_time += dt; }
    void compensate_ego_rotation(const Mat<3, 3>& T);
    void compensate_ego_yaw(double dpsi);
    void get_state(Vec<6>& x, Mat<6, 6>& P) const;
    bool is_reliable() const { return initialized && coast_time <= p_.max_coast_sec; }
    bool has_range_fix(std::optional<double> max_age = std::nullopt) const;
    bool has_vision_range_fix(std::optional<double> max_age = std::nullopt) const;
    bool is_finite() const;
    void mark_dirty() { fused_valid_ = false; }
    // leader_telemetry.apply_leader_velocity_hint_to_imm 의 핵심: 속도 상태를 alpha 로 끌고 속도 공분산을 shrink 배.
    bool apply_velocity_hint(const Vec<3>& rel_vel_cam, double alpha, double shrink_vel_cov);
    Mat<3, 3> default_R() const;

    const ImmParams& params() const { return p_; }

private:
    ImmParams p_;                       // filters 보다 먼저 선언 — 초기화 순서
public:
    std::array<SingleEkf, N_MODELS> filters;
    std::array<double, N_MODELS> mu;
    bool initialized = false;
    double coast_time = 0.0;
    double range_coast_time = 0.0;
    double vision_range_coast_time = 0.0;

private:
    void update_mode_probs(const std::array<double, N_MODELS>& lik);
    static bool bearing_hH(const Vec<6>& x, Vec<2>& h, Mat<2, 6>& H);
    mutable bool fused_valid_ = false;
    mutable Vec<6> fused_x_;
    mutable Mat<6, 6> fused_P_;
};

}  // namespace mars
