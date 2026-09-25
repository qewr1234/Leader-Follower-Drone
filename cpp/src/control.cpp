// control.cpp — main.py 제어 법칙의 이식. 파이썬 식의 덧셈 순서를 그대로 지킨다 (차등 검증 1e-12).
#include "mars_core/control.hpp"

#include <cmath>

namespace mars {

double clamp(double x, double lo, double hi) {
    if (!std::isfinite(x)) x = 0.0;
    return std::max(lo, std::min(hi, x));
}

Vec<4> compute_velocity_cmd(const ControlGains& g, const Vec<3>& rel, const Vec<3>& relv, double pos_cov_trace,
                            double target_distance, const std::optional<Vec<3>>& ff, const std::optional<Vec<3>>& slot) {
    Vec<4> zero;
    // 입력 검증 — clamp 는 NaN 을 0 으로 자르지만 파이썬과 같이 "정지" 로 명시
    if (!rel.finite() || !relv.finite() || (ff && !ff->finite()) || (slot && !slot->finite()) || !std::isfinite(target_distance))
        return zero;
    const double front = rel.a[0], right = rel.a[1], up = rel.a[2];
    const double v_front = relv.a[0], v_right = relv.a[1], v_up = relv.a[2];
    if (front <= 0.0) return zero;

    double e_front, e_right, e_up;
    if (slot) { e_front = slot->a[0]; e_right = slot->a[1]; e_up = slot->a[2]; }
    else { e_front = front - target_distance; e_right = right; e_up = up; }

    double scale;
    if (pos_cov_trace > g.slowdown_trace) scale = 0.55;
    else if (pos_cov_trace > g.slowdown_trace * 0.5) scale = 0.75;
    else scale = 1.0;

    const double ff_f = ff ? ff->a[0] : 0.0, ff_r = ff ? ff->a[1] : 0.0, ff_u = ff ? ff->a[2] : 0.0;
    const double cmd_forward = clamp((g.kff * ff_f + g.kp_forward * e_front + g.kd_forward * v_front) * scale, -g.max_vx, g.max_vx);
    const double cmd_right = clamp((g.kff * ff_r + g.kp_right * e_right + g.kd_right * v_right) * scale, -g.max_vy, g.max_vy);
    const double cmd_up = clamp((g.kff * ff_u + g.kp_up * e_up + g.kd_up * v_up) * scale, -g.max_vz, g.max_vz);
    const double cmd_yaw_rate = clamp(g.kp_yaw * std::atan2(right, std::max(front, 0.5)) * scale, -g.max_yaw_rate, g.max_yaw_rate);

    Vec<4> out;
    out.a[0] = cmd_forward; out.a[1] = cmd_right; out.a[2] = -cmd_up; out.a[3] = cmd_yaw_rate;
    return out;
}

Vec<4> smooth_velocity_cmd(const ControlGains& g, const Vec<4>& prev, const Vec<4>& next, double alpha, std::optional<double> dt) {
    double a = alpha;
    if (dt && *dt > 0.0) a = clamp(1.0 - std::pow(1.0 - a, *dt / g.smooth_ref_dt), 0.0, 1.0);
    Vec<4> out;
    const double lim[4] = {g.max_vx, g.max_vy, g.max_vz, g.max_yaw_rate};
    for (std::size_t i = 0; i < 4; ++i) {
        const double v = (1.0 - a) * prev.a[i] + a * next.a[i];
        out.a[i] = clamp(v, -lim[i], lim[i]);
    }
    return out;
}

Vec<3> leader_velocity_ff(const ControlGains& g, const Vec<3>& prev_ff, const std::optional<Vec<3>>& v_in, double dt) {
    Vec<3> target;
    if (v_in) {
        const Vec<3>& v = *v_in;
        const double speed = std::sqrt(dot(v, v));   // np.linalg.norm = sqrt(x·x)
        if (v.finite() && g.ff_deadband < speed && speed <= g.ff_speed_max) target = (1.0 - g.ff_deadband / speed) * v;
    }
    Vec<3> prev = prev_ff.finite() ? prev_ff : Vec<3>{};
    const double a = 1.0 - std::exp(-std::max(dt, 0.0) / std::max(g.ff_tau, 1e-3));
    return prev + a * (target - prev);
}

Vec<3> self_velocity_lpf(const ControlGains& g, const std::optional<Vec<3>>& prev, const Vec<3>& v, double dt) {
    if (!v.finite()) return prev ? *prev : Vec<3>{};
    if (!prev || g.ff_self_tau <= 0.0 || !prev->finite()) return v;
    const double a = 1.0 - std::exp(-std::max(dt, 0.0) / g.ff_self_tau);
    return *prev + a * (v - *prev);
}

Mat<3, 3> rot_body_to_ned(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Mat<3, 3> R;
    R(0, 0) = cy * cp; R(0, 1) = cy * sp * sr - sy * cr; R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp; R(1, 1) = sy * sp * sr + cy * cr; R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;     R(2, 1) = cp * sr;                R(2, 2) = cp * cr;
    return R;
}

Vec<3> level_fru_by_roll_pitch(const Vec<3>& v, double roll, double pitch) {
    Vec<3> frd; frd.a[0] = v.a[0]; frd.a[1] = v.a[1]; frd.a[2] = -v.a[2];
    const Vec<3> lv = rot_body_to_ned(roll, pitch, 0.0) * frd;
    Vec<3> out; out.a[0] = lv.a[0]; out.a[1] = lv.a[1]; out.a[2] = -lv.a[2];
    return out;
}

bool sanitize_cmd(const Vec<4>& cmd, Vec<4>& out) {
    if (cmd.finite()) { out = cmd; return true; }
    out = Vec<4>{};
    return false;
}

}  // namespace mars
