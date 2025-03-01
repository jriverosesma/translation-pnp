#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>

#include "TPnP/pose.hpp"

namespace py = pybind11;

PYBIND11_MODULE(tpnp_cpp, m) {
  m.doc() = "Python bindings for Pose class using pybind11";

  // Expose PoseSetup structure
  py::class_<pose::PoseSetup>(m, "PoseSetup")
      // Define custom constructor for binding only
      .def(py::init<double, const Eigen::Vector3d &, const Eigen::Matrix3d &,
                    const Eigen::Vector<double, 5> &, const Eigen::Vector3d &,
                    const Eigen::Vector3d &>(),
           py::arg("runway_true_heading"), py::arg("runway_ltp_lla"),
           py::arg("cam_intrinsic"), py::arg("cam_dist"), py::arg("cam_offset"),
           py::arg("cam_attitude_rpy"))
      .def_readwrite("runway_true_heading",
                     &pose::PoseSetup::runway_true_heading)
      .def_readwrite("runway_ltp_lla", &pose::PoseSetup::runway_ltp_lla)
      .def_readwrite("cam_intrinsic", &pose::PoseSetup::cam_intrinsic)
      .def_readwrite("cam_dist", &pose::PoseSetup::cam_dist)
      .def_readwrite("cam_offset", &pose::PoseSetup::cam_offset)
      .def_readwrite("cam_attitude_rpy", &pose::PoseSetup::cam_attitude_rpy)
      .def_readwrite("runway_true_heading",
                     &pose::PoseSetup::runway_true_heading)
      .def_readwrite("runway_ltp_lla", &pose::PoseSetup::runway_ltp_lla)
      .def_readwrite("cam_intrinsic", &pose::PoseSetup::cam_intrinsic)
      .def_readwrite("cam_dist", &pose::PoseSetup::cam_dist)
      .def_readwrite("cam_offset", &pose::PoseSetup::cam_offset)
      .def_readwrite("cam_attitude_rpy", &pose::PoseSetup::cam_attitude_rpy);

  // Expose PoseInput structure
  py::class_<pose::PoseInput>(m, "PoseInput")
      .def(py::init<const Eigen::Matrix<double, Eigen::Dynamic, 2> &,
                    const Eigen::Matrix<double, Eigen::Dynamic, 3> &,
                    const std::optional<Eigen::Vector3d> &>(),
           py::arg("points_2d"), py::arg("points_3d"),
           py::arg("attitude") = std::nullopt)
      .def_readwrite("points_2d", &pose::PoseInput::points_2d)
      .def_readwrite("points_3d", &pose::PoseInput::points_3d)
      .def_readwrite("ac_attitude_rpy", &pose::PoseInput::ac_attitude_rpy);

  // Expose Pose class and its methods
  py::class_<pose::Pose>(m, "Pose")
      .def(py::init<>())  // Default constructor
      .def("setup", &pose::Pose::setup, py::arg("pose_setup"))
      .def("compute_pose", &pose::Pose::compute_pose, py::arg("pose_input"))
      .def("solve_tpnp", &pose::Pose::solve_tpnp, py::arg("points_2d"),
           py::arg("points_3d"), py::arg("ac_attitude_rpy"))
      .def("solve_opencv_pnp", &pose::Pose::solve_opencv_pnp,
           py::arg("points_2d"), py::arg("points_3d"));
}
