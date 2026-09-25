// control.hpp — main.py 의 제어 법칙·필터 이식: compute_velocity_cmd_from_estimate, smooth_velocity_cmd,
// leader_velocity_ff, self_velocity_lpf, level_fru_by_roll_pitch, sanitize_cmd, clamp.
// 이득·한계는 호출마다 ControlGains 로 받는다 (파이썬 쪽이 main 의 모듈 상수를 그때그때 읽어 넘긴다 — 분석 스크립트가
// 상수를 바꿔 가며 스윕하는 경우도 그대로 통한다).
#pragma once

#include <optional>

#include "mars_core/mat.hpp"

namespace mars {

struct ControlGains {
    double kp_forward = 0.22, kd_forward = 0.05;
    double kp_right = 0.28, kd_right = 0.04;
    double kp_up = 0.18, kd_up = 0.03;
    double kff = 0.8;
    double kp_yaw = 0.8;
    double max_vx = 0.35, max_vy = 0.22, max_vz = 0.12, max_yaw_rate = 0.35;
    double target_distance = 3.0;
    double slowdown_trace = 4.0;        // UNCERTAINTY_SLOWDOWN_TRACE
    double ff_tau = 2.0;                // FF_TAU_SEC
    double ff_self_tau = 0.3;           // FF_SELF_TAU_SEC
    double ff_deadband = 0.05;          // FF_DEADBAND_MPS
    double ff_speed_max = 20.0;         // leader_velocity_ff 의 리더 속도 상한
    double smooth_ref_dt = 1.0 / 30.0;  // SMOOTH_REF_DT
};

// utils_geometry.clamp: NaN/inf → 0 을 범위로 자른 값
double clamp(double x, double lo, double hi);

// main.compute_velocity_cmd_from_estimate → BODY_NED [vx, vy, vz(down+), yaw_rate]
Vec<4> compute_velocity_cmd(const ControlGains& g, const Vec<3>& rel_fru, const Vec<3>& rel_vel_fru, double pos_cov_trace,
                            double target_distance, const std::optional<Vec<3>>& leader_vel_ff,
                            const std::optional<Vec<3>>& slot_error);

// main.smooth_velocity_cmd (dt 없음 = alpha 그대로)
Vec<4> smooth_velocity_cmd(const ControlGains& g, const Vec<4>& prev, const Vec<4>& next, double alpha, std::optional<double> dt);

// main.leader_velocity_ff — 소프트 데드존 → 1차 저역통과. v 가 없으면 0 으로 감쇠
Vec<3> leader_velocity_ff(const ControlGains& g, const Vec<3>& prev_ff, const std::optional<Vec<3>>& leader_vel_fru, double dt);

// main.self_velocity_lpf — prev 없음(nullopt) 이면 현재 값으로 시작
Vec<3> self_velocity_lpf(const ControlGains& g, const std::optional<Vec<3>>& prev, const Vec<3>& v_self_fru, double dt);

// main.rot_body_to_ned / level_fru_by_roll_pitch
Mat<3, 3> rot_body_to_ned(double roll, double pitch, double yaw);
Vec<3> level_fru_by_roll_pitch(const Vec<3>& v_fru, double roll, double pitch);

// main.sanitize_cmd — 비유한이면 (0, false)
bool sanitize_cmd(const Vec<4>& cmd, Vec<4>& out);

}  // namespace mars
