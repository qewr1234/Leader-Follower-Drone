// imm_ekf.cpp — imm_ekf.py 의 이식. 각 함수 위에 원본 파이썬 함수명을 적었다.
#include "mars_core/imm_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mars {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// _F_cv(dt)
Mat<6, 6> F_cv(double dt) {
    Mat<6, 6> F = Mat<6, 6>::identity();
    F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;
    return F;
}

// _Q(dt, sigma_a) — 등가속 백색잡음, 세 축 동일·독립
Mat<6, 6> Q_of(double dt, double sigma_a) {
    const double q = sigma_a * sigma_a;
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    Mat<6, 6> Q;
    for (std::size_t i = 0; i < 3; ++i) {
        Q(i, i) += dt4 / 4 * q;
        Q(i, i + 3) += dt3 / 2 * q;
        Q(i + 3, i) += dt3 / 2 * q;
        Q(i + 3, i + 3) += dt2 * q;
    }
    return Q;
}

// _F_ct(dt, omega) — 카메라 x-z 수평면 coordinated turn
Mat<6, 6> F_ct(double dt, double omega) {
    if (std::fabs(omega) < 1e-5) return F_cv(dt);
    const double s = std::sin(omega * dt), c = std::cos(omega * dt);
    const double so = s / omega, co = (1 - c) / omega;
    Mat<6, 6> F = Mat<6, 6>::identity();
    F(0, 3) = so; F(0, 5) = -co;
    F(2, 3) = co; F(2, 5) = so;
    F(3, 3) = c;  F(3, 5) = -s;
    F(5, 3) = s;  F(5, 5) = c;
    F(1, 4) = dt;
    return F;
}

Mat<3, 6> H_pos() {
    Mat<3, 6> H;
    H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0;
    return H;
}

// _gaussian_likelihood(y, S, inv_S, dim)
template <std::size_t D>
double gaussian_likelihood(const Vec<D>& y, const Mat<D, D>& S, const Mat<D, D>& inv_S) {
    const double det_S = determinant(S);
    if (det_S <= 0) return 1e-300;
    const Vec<D> Sy = inv_S * y;
    const double exp_term = std::max(-0.5 * dot(y, Sy), -500.0);
    double pw = 1.0;
    for (std::size_t i = 0; i < D; ++i) pw *= kTwoPi;
    const double lik = std::exp(exp_term) / std::sqrt(pw * det_S);
    return std::max(lik, 1e-300);
}

}  // namespace

// ---------------------------------------------------------------- _SingleEKF
SingleEkf::SingleEkf(int id, const ImmParams& p) : model_id(id), p_(p) {
    for (std::size_t i = 0; i < 3; ++i) { P(i, i) = p.p0_pos; P(i + 3, i + 3) = p.p0_vel; }
}

void SingleEkf::reset(const Vec<6>& x0, const Mat<6, 6>& P0) { x = x0; P = P0; }

void SingleEkf::predict(double dt) {
    dt_since_update += dt;
    Mat<6, 6> F, Q;
    if (model_id == 0) { F = F_cv(dt); Q = Q_of(dt, p_.sigma_a_cv); }
    else               { F = F_ct(dt, omega); Q = Q_of(dt, p_.sigma_a_ct); }
    x = F * x;
    P = F * P * F.transpose() + Q;
}

double SingleEkf::update_position3d(const Vec<3>& z, const Mat<3, 3>& R) {
    const Mat<3, 6> H = H_pos();
    return update<3>(z, H * x, H, R);
}

double SingleEkf::update_bearing2d(const Vec<2>& z, const Mat<2, 2>& R) {
    const double px = x.a[0], py = x.a[1], pz = x.a[2];
    if (pz <= 1e-4) return 1e-300;
    Vec<2> h; h.a[0] = px / pz; h.a[1] = py / pz;
    Mat<2, 6> H;
    H(0, 0) = 1.0 / pz; H(0, 2) = -px / (pz * pz);
    H(1, 1) = 1.0 / pz; H(1, 2) = -py / (pz * pz);
    return update<2>(z, h, H, R);
}

// _update(z, h, H, R, dim) — Joseph 형
template <std::size_t D>
double SingleEkf::update(const Vec<D>& z, const Vec<D>& h, const Mat<D, 6>& H, const Mat<D, D>& R) {
    const Vec<D> y = z - h;
    const Mat<D, D> S = H * P * H.transpose() + R;
    Mat<D, D> inv_S;
    if (!inverse(S, inv_S)) return 1e-300;
    const Mat<6, D> K = P * H.transpose() * inv_S;
    x = x + K * y;
    const Mat<6, 6> I_KH = Mat<6, 6>::identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    if (model_id == 1) estimate_omega();
    return gaussian_likelihood(y, S, inv_S);
}

// _estimate_omega
void SingleEkf::estimate_omega() {
    const double vx = x.a[3], vz = x.a[5];
    if (std::hypot(vx, vz) > 0.3) {
        const double heading = std::atan2(vz, vx);
        if (prev_heading && dt_since_update > 1e-3) {
            const double d_heading = wrap_pi(heading - *prev_heading);
            const double omega_raw = std::clamp(d_heading / dt_since_update, -1.5, 1.5);
            omega = 0.7 * omega + 0.3 * omega_raw;
        }
        prev_heading = heading;
    } else {
        prev_heading.reset();
        omega *= 0.9;
    }
    dt_since_update = 0.0;
}

// ---------------------------------------------------------------- ImmEkf
ImmEkf::ImmEkf(const ImmParams& p) : p_(p), filters{SingleEkf(0, p), SingleEkf(1, p)}, mu(p.mu0) {}

Mat<3, 3> ImmEkf::default_R() const {
    Mat<3, 3> R;
    R(0, 0) = p_.sigma_xy * p_.sigma_xy; R(1, 1) = p_.sigma_xy * p_.sigma_xy; R(2, 2) = p_.sigma_z * p_.sigma_z;
    return R;
}

void ImmEkf::init(const Vec<3>& z, bool source_rgbd) {
    Vec<6> x0; x0.a[0] = z.a[0]; x0.a[1] = z.a[1]; x0.a[2] = z.a[2];
    Mat<6, 6> P0;
    for (std::size_t i = 0; i < 3; ++i) { P0(i, i) = p_.p0_pos; P0(i + 3, i + 3) = p_.p0_vel; }
    for (auto& f : filters) f.reset(x0, P0);
    mu = p_.mu0;
    initialized = true;
    coast_time = 0.0;
    range_coast_time = 0.0;
    vision_range_coast_time = source_rgbd ? 0.0 : std::numeric_limits<double>::infinity();
    fused_valid_ = false;
}

void ImmEkf::reset() {
    filters = {SingleEkf(0, p_), SingleEkf(1, p_)};
    mu = p_.mu0;
    initialized = false;
    coast_time = range_coast_time = vision_range_coast_time = 0.0;
    fused_valid_ = false;
}

void ImmEkf::predict(double dt) {
    if (!initialized) return;
    dt = std::max(dt, 1e-4);
    range_coast_time += dt;
    vision_range_coast_time += dt;

    const auto& Pi = p_.trans_prob;
    std::array<double, N_MODELS> mu_pred{};
    for (int j = 0; j < N_MODELS; ++j) {
        double s = 0.0;
        for (int i = 0; i < N_MODELS; ++i) s += Pi[i][j] * mu[i];   // Pi.T @ mu
        mu_pred[j] = s;
    }
    const double mp_sum = std::max(mu_pred[0] + mu_pred[1], 1e-300);
    for (auto& v : mu_pred) v /= mp_sum;

    std::array<Vec<6>, N_MODELS> xs;
    std::array<Mat<6, 6>, N_MODELS> Ps;
    for (int j = 0; j < N_MODELS; ++j) {
        std::array<double, N_MODELS> c{};
        for (int i = 0; i < N_MODELS; ++i) c[i] = Pi[i][j] * mu[i];
        const double cs = std::max(c[0] + c[1], 1e-300);
        for (auto& v : c) v /= cs;
        Vec<6> x_j;                                   // sum(c_ij[i] * x_i) — 0 부터 i 순서
        for (int i = 0; i < N_MODELS; ++i) x_j = x_j + c[i] * filters[i].x;
        Mat<6, 6> P_j;
        for (int i = 0; i < N_MODELS; ++i) {
            const Vec<6> dx = filters[i].x - x_j;
            P_j = P_j + c[i] * (filters[i].P + outer(dx, dx));
        }
        xs[j] = x_j; Ps[j] = P_j;
    }
    for (int j = 0; j < N_MODELS; ++j) { filters[j].reset(xs[j], Ps[j]); filters[j].predict(dt); }
    mu = mu_pred;
    fused_valid_ = false;
}

void ImmEkf::update_position3d(const Vec<3>& z, const Mat<3, 3>* R, bool source_rgbd) {
    if (!initialized) { init(z, source_rgbd); return; }
    coast_time = 0.0;
    range_coast_time = 0.0;
    if (source_rgbd) vision_range_coast_time = 0.0;
    const Mat<3, 3> Ruse = R ? *R : default_R();
    std::array<double, N_MODELS> lik{};
    for (int i = 0; i < N_MODELS; ++i) lik[i] = filters[i].update_position3d(z, Ruse);
    update_mode_probs(lik);
}

void ImmEkf::update_bearing2d(const Vec<2>& z, const Mat<2, 2>& R) {
    if (!initialized) return;
    coast_time = 0.0;
    std::array<double, N_MODELS> lik{};
    for (int i = 0; i < N_MODELS; ++i) lik[i] = filters[i].update_bearing2d(z, R);
    update_mode_probs(lik);
}

void ImmEkf::update_mode_probs(const std::array<double, N_MODELS>& lik) {
    std::array<double, N_MODELS> m{};
    double total = 0.0;
    for (int i = 0; i < N_MODELS; ++i) { m[i] = mu[i] * lik[i]; total += m[i]; }
    if (total < 1e-300 || !std::isfinite(total)) { for (auto& v : mu) v = 1.0 / N_MODELS; }
    else { for (int i = 0; i < N_MODELS; ++i) mu[i] = m[i] / total; }
    fused_valid_ = false;
}

bool ImmEkf::bearing_hH(const Vec<6>& x, Vec<2>& h, Mat<2, 6>& H) {
    const double px = x.a[0], py = x.a[1], pz = x.a[2];
    if (pz <= 1e-4) return false;
    h.a[0] = px / pz; h.a[1] = py / pz;
    H = Mat<2, 6>{};
    H(0, 0) = 1.0 / pz; H(0, 2) = -px / (pz * pz);
    H(1, 1) = 1.0 / pz; H(1, 2) = -py / (pz * pz);
    return true;
}

void ImmEkf::innovation_position3d(const Vec<3>& z, const Mat<3, 3>* R, Vec<3>& y, Mat<3, 3>& S) const {
    if (!initialized) { y = Vec<3>{}; S = 999.0 * Mat<3, 3>::identity(); return; }
    Vec<6> x; Mat<6, 6> P; get_state(x, P);
    const Mat<3, 6> H = H_pos();
    const Mat<3, 3> Ruse = R ? *R : default_R();
    y = z - H * x;
    S = H * P * H.transpose() + Ruse;
}

void ImmEkf::innovation_bearing2d(const Vec<2>& z, const Mat<2, 2>& R, Vec<2>& y, Mat<2, 2>& S) const {
    if (!initialized) { y = Vec<2>{}; S = 999.0 * Mat<2, 2>::identity(); return; }
    Vec<6> x; Mat<6, 6> P; get_state(x, P);
    Vec<2> h; Mat<2, 6> H;
    if (!bearing_hH(x, h, H)) { y.a[0] = 999.0; y.a[1] = 999.0; S = 999.0 * Mat<2, 2>::identity(); return; }
    y = z - h;
    S = H * P * H.transpose() + R;
}

void ImmEkf::compensate_ego_rotation(const Mat<3, 3>& T) {
    if (!initialized) return;
    double maxdev = 0.0;
    const Mat<3, 3> I = Mat<3, 3>::identity();
    for (std::size_t i = 0; i < 9; ++i) maxdev = std::max(maxdev, std::fabs(T.a[i] - I.a[i]));
    if (maxdev < 1e-9) return;
    Mat<6, 6> G;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) { G(i, j) = T(i, j); G(i + 3, j + 3) = T(i, j); }
    for (auto& f : filters) {
        std::optional<double> h0;
        if (f.prev_heading) h0 = std::atan2(f.x.a[5], f.x.a[3]);
        f.x = G * f.x;
        f.P = G * f.P * G.transpose();
        if (h0) f.prev_heading = wrap_pi(*f.prev_heading + wrap_pi(std::atan2(f.x.a[5], f.x.a[3]) - *h0));
    }
    fused_valid_ = false;
}

void ImmEkf::compensate_ego_yaw(double dpsi) {
    if (std::fabs(dpsi) < 1e-6) return;
    const double c = std::cos(dpsi), s = std::sin(dpsi);
    Mat<3, 3> T;
    T(0, 0) = c; T(0, 2) = -s; T(1, 1) = 1.0; T(2, 0) = s; T(2, 2) = c;
    compensate_ego_rotation(T);
}

void ImmEkf::get_state(Vec<6>& x, Mat<6, 6>& P) const {
    if (!initialized) { x = Vec<6>{}; P = 999.0 * Mat<6, 6>::identity(); return; }
    if (!fused_valid_) {
        Vec<6> xf;
        for (int i = 0; i < N_MODELS; ++i) xf = xf + mu[i] * filters[i].x;
        Mat<6, 6> Pf;
        for (int i = 0; i < N_MODELS; ++i) {
            const Vec<6> dx = filters[i].x - xf;
            Pf = Pf + mu[i] * (filters[i].P + outer(dx, dx));
        }
        fused_x_ = xf; fused_P_ = Pf; fused_valid_ = true;
    }
    x = fused_x_; P = fused_P_;
}

bool ImmEkf::has_range_fix(std::optional<double> max_age) const {
    const double limit = max_age ? *max_age : p_.range_coast_max_sec;
    return initialized && range_coast_time <= limit;
}

bool ImmEkf::has_vision_range_fix(std::optional<double> max_age) const {
    const double limit = max_age ? *max_age : p_.range_coast_max_sec;
    return initialized && vision_range_coast_time <= limit;
}

bool ImmEkf::is_finite() const {
    Vec<6> x; Mat<6, 6> P; get_state(x, P);
    return x.finite() && P.finite();
}

bool ImmEkf::apply_velocity_hint(const Vec<3>& v, double alpha, double shrink) {
    if (!initialized) return false;
    if (!v.finite()) return false;
    for (auto& f : filters) {
        for (std::size_t k = 0; k < 3; ++k) f.x.a[3 + k] = (1.0 - alpha) * f.x.a[3 + k] + alpha * v.a[k];
        for (std::size_t i = 3; i < 6; ++i)
            for (std::size_t j = 3; j < 6; ++j) f.P(i, j) *= shrink;
    }
    fused_valid_ = false;
    return true;
}

}  // namespace mars
