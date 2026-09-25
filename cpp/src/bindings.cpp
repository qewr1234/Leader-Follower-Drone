// bindings.cpp — pybind11 모듈 `mars_core`. 파이썬 imm_ekf.ImmEkf 와 같은 메서드 이름·반환 형태를 지킨다
// (main.py / reliability.py / scheduler.py / leader_telemetry.py 가 그대로 쓴다).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mars_core/control.hpp"
#include "mars_core/imm_ekf.hpp"
#include "mars_core/mission.hpp"

namespace py = pybind11;
using namespace mars;

namespace {

template <std::size_t N>
Vec<N> to_vec(const py::array_t<double, py::array::c_style | py::array::forcecast>& a) {
    if (a.size() < static_cast<py::ssize_t>(N)) throw std::invalid_argument("vector too short");
    Vec<N> v;
    auto r = a.unchecked<>();
    if (r.ndim() == 1) for (std::size_t i = 0; i < N; ++i) v.a[i] = r(i);
    else if (r.ndim() == 2) for (std::size_t i = 0; i < N; ++i) v.a[i] = r(i, 0);
    else throw std::invalid_argument("bad ndim");
    return v;
}

template <std::size_t R, std::size_t C>
Mat<R, C> to_mat(const py::array_t<double, py::array::c_style | py::array::forcecast>& a) {
    if (a.ndim() != 2 || a.shape(0) != static_cast<py::ssize_t>(R) || a.shape(1) != static_cast<py::ssize_t>(C))
        throw std::invalid_argument("bad matrix shape");
    Mat<R, C> m;
    auto r = a.unchecked<2>();
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j) m(i, j) = r(i, j);
    return m;
}

template <std::size_t N>
py::array_t<double> from_vec(const Vec<N>& v) {
    py::array_t<double> a(N);
    auto w = a.mutable_unchecked<1>();
    for (std::size_t i = 0; i < N; ++i) w(i) = v.a[i];
    return a;
}

template <std::size_t R, std::size_t C>
py::array_t<double> from_mat(const Mat<R, C>& m) {
    py::array_t<double> a({static_cast<py::ssize_t>(R), static_cast<py::ssize_t>(C)});
    auto w = a.mutable_unchecked<2>();
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j) w(i, j) = m(i, j);
    return a;
}

using NpArr = py::array_t<double, py::array::c_style | py::array::forcecast>;

// None → nullopt, 아니면 앞 N 성분 (파이썬 쪽 np.asarray(x)[:N] 과 같은 관대함)
template <std::size_t N>
std::optional<Vec<N>> opt_vec(const py::object& o) {
    if (o.is_none()) return std::nullopt;
    return to_vec<N>(o.cast<NpArr>());
}

std::optional<double> opt_double(const py::object& o) {
    if (o.is_none()) return std::nullopt;
    return o.cast<double>();
}

py::object from_opt_double(const std::optional<double>& v) {
    if (!v) return py::none();
    return py::float_(*v);
}

py::dict policy_dict(const MissionPolicy& p) {
    py::dict d;
    d["mode"] = p.mode; d["allow_follow"] = p.allow_follow; d["land"] = p.land;
    return d;
}

py::array_t<double> mu_array(const ImmEkf& e) {
    py::array_t<double> a(2);
    auto w = a.mutable_unchecked<1>();
    w(0) = e.mu[0]; w(1) = e.mu[1];
    return a;
}

}  // namespace

PYBIND11_MODULE(mars_core, m) {
    m.doc() = "MARS-IMM C++ core (IMM-EKF). imm_ekf.py 와 차등 검증됨.";

    py::class_<ImmEkf>(m, "ImmEkf")
        .def(py::init([](double sigma_xy, double sigma_z, double max_coast_sec, double range_coast_max_sec) {
                 ImmParams p;
                 p.sigma_xy = sigma_xy; p.sigma_z = sigma_z;
                 p.max_coast_sec = max_coast_sec; p.range_coast_max_sec = range_coast_max_sec;
                 return new ImmEkf(p);
             }),
             py::arg("sigma_xy") = 0.15, py::arg("sigma_z") = 0.25, py::arg("max_coast_sec") = 2.0,
             py::arg("range_coast_max_sec") = 2.0)
        .def("init", [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> z, const std::string& source) {
                 e.init(to_vec<3>(z), source == "rgbd");
             }, py::arg("z"), py::arg("source") = "rgbd")
        .def("reset", &ImmEkf::reset)
        .def("predict", &ImmEkf::predict, py::arg("dt"))
        .def("update_position3d", [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> z, py::object R, const std::string& source) {
                 if (R.is_none()) e.update_position3d(to_vec<3>(z), nullptr, source == "rgbd");
                 else { const Mat<3, 3> Rm = to_mat<3, 3>(R.cast<py::array_t<double, py::array::c_style | py::array::forcecast>>()); e.update_position3d(to_vec<3>(z), &Rm, source == "rgbd"); }
             }, py::arg("z"), py::arg("R") = py::none(), py::arg("source") = "rgbd")
        .def("update_bearing2d", [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> z, py::array_t<double, py::array::c_style | py::array::forcecast> R) {
                 e.update_bearing2d(to_vec<2>(z), to_mat<2, 2>(R));
             }, py::arg("z"), py::arg("R"))
        .def("innovation_position3d", [](const ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> z, py::object R) {
                 Vec<3> y; Mat<3, 3> S;
                 if (R.is_none()) e.innovation_position3d(to_vec<3>(z), nullptr, y, S);
                 else { const Mat<3, 3> Rm = to_mat<3, 3>(R.cast<py::array_t<double, py::array::c_style | py::array::forcecast>>()); e.innovation_position3d(to_vec<3>(z), &Rm, y, S); }
                 return py::make_tuple(from_vec(y), from_mat(S));
             }, py::arg("z"), py::arg("R") = py::none())
        .def("innovation_bearing2d", [](const ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> z, py::array_t<double, py::array::c_style | py::array::forcecast> R) {
                 Vec<2> y; Mat<2, 2> S;
                 e.innovation_bearing2d(to_vec<2>(z), to_mat<2, 2>(R), y, S);
                 return py::make_tuple(from_vec(y), from_mat(S));
             }, py::arg("z"), py::arg("R"))
        .def("on_lost", &ImmEkf::on_lost, py::arg("dt"))
        .def("compensate_ego_rotation", [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> T) { e.compensate_ego_rotation(to_mat<3, 3>(T)); }, py::arg("T"))
        .def("compensate_ego_yaw", &ImmEkf::compensate_ego_yaw, py::arg("dpsi"))
        .def("get_state", [](const ImmEkf& e) { Vec<6> x; Mat<6, 6> P; e.get_state(x, P); return py::make_tuple(from_vec(x), from_mat(P)); })
        .def("get_state_dict", [](const ImmEkf& e) {
                 Vec<6> x; Mat<6, 6> P; e.get_state(x, P);
                 py::dict d;
                 d["x"] = from_vec(x); d["P"] = from_mat(P); d["mode_probs"] = mu_array(e);
                 d["initialized"] = e.initialized; d["coast_time"] = e.coast_time;
                 d["range_coast_time"] = e.range_coast_time; d["vision_range_coast_time"] = e.vision_range_coast_time;
                 return d;
             })
        .def("get_model_probs", [](const ImmEkf& e) { return mu_array(e); })
        .def("is_reliable", &ImmEkf::is_reliable)
        .def("has_range_fix", [](const ImmEkf& e, py::object max_age) {
                 return max_age.is_none() ? e.has_range_fix() : e.has_range_fix(max_age.cast<double>());
             }, py::arg("max_age") = py::none())
        .def("has_vision_range_fix", [](const ImmEkf& e, py::object max_age) {
                 return max_age.is_none() ? e.has_vision_range_fix() : e.has_vision_range_fix(max_age.cast<double>());
             }, py::arg("max_age") = py::none())
        .def("is_finite", &ImmEkf::is_finite)
        .def("mark_dirty", &ImmEkf::mark_dirty)
        .def("apply_velocity_hint", [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> v, double alpha, double shrink) {
                 return e.apply_velocity_hint(to_vec<3>(v), alpha, shrink);
             }, py::arg("rel_vel_cam"), py::arg("alpha") = 0.12, py::arg("shrink_vel_cov") = 0.96)
        // 필터 내부 접근 (검사·차등 검증용)
        .def("get_filter_x", [](const ImmEkf& e, int i) { return from_vec(e.filters.at(i).x); })
        .def("get_filter_P", [](const ImmEkf& e, int i) { return from_mat(e.filters.at(i).P); })
        .def("get_filter_omega", [](const ImmEkf& e, int i) { return e.filters.at(i).omega; })
        .def("set_filter_x", [](ImmEkf& e, int i, py::array_t<double, py::array::c_style | py::array::forcecast> x) { e.filters.at(i).x = to_vec<6>(x); e.mark_dirty(); })
        .def("set_filter_P", [](ImmEkf& e, int i, py::array_t<double, py::array::c_style | py::array::forcecast> P) { e.filters.at(i).P = to_mat<6, 6>(P); e.mark_dirty(); })
        .def_property("mu", [](const ImmEkf& e) { return mu_array(e); },
                      [](ImmEkf& e, py::array_t<double, py::array::c_style | py::array::forcecast> m) { auto r = m.unchecked<1>(); e.mu[0] = r(0); e.mu[1] = r(1); e.mark_dirty(); })
        .def_readwrite("initialized", &ImmEkf::initialized)
        .def_readwrite("coast_time", &ImmEkf::coast_time)
        .def_readwrite("range_coast_time", &ImmEkf::range_coast_time)
        .def_readwrite("vision_range_coast_time", &ImmEkf::vision_range_coast_time);

    // ---------------- 제어 법칙 (main.py 의 함수들; 이득은 ControlGains 로 매 호출 전달) ----------------
    py::class_<ControlGains>(m, "ControlGains")
        .def(py::init<>())
        .def_readwrite("kp_forward", &ControlGains::kp_forward).def_readwrite("kd_forward", &ControlGains::kd_forward)
        .def_readwrite("kp_right", &ControlGains::kp_right).def_readwrite("kd_right", &ControlGains::kd_right)
        .def_readwrite("kp_up", &ControlGains::kp_up).def_readwrite("kd_up", &ControlGains::kd_up)
        .def_readwrite("kff", &ControlGains::kff).def_readwrite("kp_yaw", &ControlGains::kp_yaw)
        .def_readwrite("max_vx", &ControlGains::max_vx).def_readwrite("max_vy", &ControlGains::max_vy)
        .def_readwrite("max_vz", &ControlGains::max_vz).def_readwrite("max_yaw_rate", &ControlGains::max_yaw_rate)
        .def_readwrite("target_distance", &ControlGains::target_distance).def_readwrite("slowdown_trace", &ControlGains::slowdown_trace)
        .def_readwrite("ff_tau", &ControlGains::ff_tau).def_readwrite("ff_self_tau", &ControlGains::ff_self_tau)
        .def_readwrite("ff_deadband", &ControlGains::ff_deadband).def_readwrite("ff_speed_max", &ControlGains::ff_speed_max)
        .def_readwrite("smooth_ref_dt", &ControlGains::smooth_ref_dt);
    m.def("clamp", &mars::clamp, py::arg("x"), py::arg("lo"), py::arg("hi"));
    m.def("compute_velocity_cmd", [](const ControlGains& g, NpArr rel, NpArr relv, double pos_cov_trace, py::object target_distance,
                                     py::object leader_vel_ff, py::object slot_error) {
              const double td = target_distance.is_none() ? g.target_distance : target_distance.cast<double>();
              return from_vec(compute_velocity_cmd(g, to_vec<3>(rel), to_vec<3>(relv), pos_cov_trace, td, opt_vec<3>(leader_vel_ff), opt_vec<3>(slot_error)));
          }, py::arg("gains"), py::arg("rel_fru"), py::arg("rel_vel_fru"), py::arg("pos_cov_trace"), py::arg("target_distance") = py::none(),
          py::arg("leader_vel_ff") = py::none(), py::arg("slot_error") = py::none());
    m.def("smooth_velocity_cmd", [](const ControlGains& g, NpArr prev, NpArr next, double alpha, py::object dt) {
              return from_vec(smooth_velocity_cmd(g, to_vec<4>(prev), to_vec<4>(next), alpha, opt_double(dt)));
          }, py::arg("gains"), py::arg("prev_cmd"), py::arg("new_cmd"), py::arg("alpha") = 0.28, py::arg("dt") = py::none());
    m.def("leader_velocity_ff", [](const ControlGains& g, NpArr prev_ff, py::object leader_vel_fru, double dt) {
              return from_vec(leader_velocity_ff(g, to_vec<3>(prev_ff), opt_vec<3>(leader_vel_fru), dt));
          }, py::arg("gains"), py::arg("prev_ff"), py::arg("leader_vel_fru"), py::arg("dt"));
    m.def("self_velocity_lpf", [](const ControlGains& g, py::object prev, NpArr v, double dt) {
              return from_vec(self_velocity_lpf(g, opt_vec<3>(prev), to_vec<3>(v), dt));
          }, py::arg("gains"), py::arg("prev"), py::arg("v_self_fru"), py::arg("dt"));
    m.def("rot_body_to_ned", [](double roll, double pitch, double yaw) { return from_mat(rot_body_to_ned(roll, pitch, yaw)); },
          py::arg("roll"), py::arg("pitch"), py::arg("yaw"));
    m.def("level_fru_by_roll_pitch", [](NpArr v, double roll, double pitch) { return from_vec(level_fru_by_roll_pitch(to_vec<3>(v), roll, pitch)); },
          py::arg("v_fru"), py::arg("roll"), py::arg("pitch"));
    m.def("sanitize_cmd", [](NpArr cmd) {
              Vec<4> out; bool ok = false;
              if (cmd.ndim() == 1 && cmd.shape(0) == 4) ok = sanitize_cmd(to_vec<4>(cmd), out);
              return py::make_tuple(from_vec(out), ok);
          }, py::arg("cmd"));

    // ---------------- 미션 상태머신 (mission_manager.MissionManager 와 같은 이름·인자·반환) ----------------
    py::class_<MissionManager>(m, "MissionManager")
        .def(py::init([](double start_speed_thresh, double start_confirm_sec, double hover_speed_thresh, double landing_z_thresh,
                         double landing_vz_thresh, double landing_hspeed_thresh, double landing_confirm_sec, double lost_hold_sec) {
                 MissionParams p;
                 p.start_speed_thresh = start_speed_thresh; p.start_confirm_sec = start_confirm_sec; p.hover_speed_thresh = hover_speed_thresh;
                 p.landing_z_thresh = landing_z_thresh; p.landing_vz_thresh = landing_vz_thresh; p.landing_hspeed_thresh = landing_hspeed_thresh;
                 p.landing_confirm_sec = landing_confirm_sec; p.lost_hold_sec = lost_hold_sec;
                 return MissionManager(p);
             }),
             py::arg("start_speed_thresh") = 0.25, py::arg("start_confirm_sec") = 0.7, py::arg("hover_speed_thresh") = 0.18,
             py::arg("landing_z_thresh") = 0.65, py::arg("landing_vz_thresh") = -0.10, py::arg("landing_hspeed_thresh") = 0.25,
             py::arg("landing_confirm_sec") = 1.8, py::arg("lost_hold_sec") = 8.0)
        .def("reset", &MissionManager::reset)
        .def("update", [](MissionManager& mm, double now, bool leader_visible, py::object rel_est, py::object rel_vel_est, py::object leader_alt,
                          py::object leader_vel_world, double pos_cov_trace, py::object leader_vel_body) {
                 const MissionState s = mm.update(now, leader_visible, opt_vec<3>(rel_est), opt_vec<3>(rel_vel_est), opt_double(leader_alt),
                                                  opt_vec<3>(leader_vel_world), pos_cov_trace, opt_vec<3>(leader_vel_body));
                 return py::make_tuple(py::str(mission_state_name(s)), policy_dict(mm.command_policy()));
             },
             py::arg("now"), py::arg("leader_visible"), py::arg("rel_est") = py::none(), py::arg("rel_vel_est") = py::none(),
             py::arg("leader_alt") = py::none(), py::arg("leader_vel_world") = py::none(), py::arg("pos_cov_trace") = 999.0,
             py::arg("leader_vel_body") = py::none())
        .def("command_policy", [](const MissionManager& mm) { return policy_dict(mm.command_policy()); })
        .def_property("state", [](const MissionManager& mm) { return std::string(mission_state_name(mm.state)); },
                      [](MissionManager& mm, const std::string& name) {
                          const auto s = mission_state_from_name(name);
                          if (!s) throw std::invalid_argument("unknown mission state: " + name);
                          mm.state = *s;
                      })
        .def_property("last_seen_t", [](const MissionManager& mm) { return from_opt_double(mm.last_seen_t); },
                      [](MissionManager& mm, py::object v) { mm.last_seen_t = opt_double(v); })
        .def_property("start_candidate_t", [](const MissionManager& mm) { return from_opt_double(mm.start_candidate_t); },
                      [](MissionManager& mm, py::object v) { mm.start_candidate_t = opt_double(v); })
        .def_property("landing_candidate_t", [](const MissionManager& mm) { return from_opt_double(mm.landing_candidate_t); },
                      [](MissionManager& mm, py::object v) { mm.landing_candidate_t = opt_double(v); })
        .def_readwrite("has_followed", &MissionManager::has_followed)
#define MARS_PARAM(name) .def_property(#name, [](const MissionManager& mm) { return mm.params().name; }, [](MissionManager& mm, double v) { mm.params_mut().name = v; })
        MARS_PARAM(start_speed_thresh) MARS_PARAM(start_confirm_sec) MARS_PARAM(hover_speed_thresh) MARS_PARAM(landing_z_thresh)
        MARS_PARAM(landing_vz_thresh) MARS_PARAM(landing_hspeed_thresh) MARS_PARAM(landing_confirm_sec) MARS_PARAM(lost_hold_sec)
#undef MARS_PARAM
        ;
    m.attr("MISSION_STATES") = py::make_tuple("WAIT_LEADER", "READY_HOVER", "FOLLOW", "LEADER_HOVER", "LANDING_CANDIDATE",
                                              "CONFIRMED_LANDING", "LOST_HOLD", "FAILSAFE_LAND");

    m.attr("__version__") = "0.2.0";
}
