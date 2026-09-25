// bindings.cpp — pybind11 모듈 `mars_core`. 파이썬 imm_ekf.ImmEkf 와 같은 메서드 이름·반환 형태를 지킨다
// (main.py / reliability.py / scheduler.py / leader_telemetry.py 가 그대로 쓴다).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mars_core/imm_ekf.hpp"

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

    m.attr("__version__") = "0.1.0";
}
