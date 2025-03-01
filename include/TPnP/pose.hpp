#ifndef POSE
#define POSE

#include <Eigen/Dense>
#include <optional>

namespace pose {
struct PoseSetup {
  double runway_true_heading;
  Eigen::Vector3d runway_ltp_lla;
  Eigen::Matrix3d cam_intrinsic;
  Eigen::Vector<double, 5> cam_dist;
  Eigen::Vector3d cam_offset;
  Eigen::Vector3d cam_attitude_rpy;
};

struct PoseInput {
  Eigen::Matrix<double, Eigen::Dynamic, 2> points_2d;
  Eigen::Matrix<double, Eigen::Dynamic, 3> points_3d;  // LLA (deg, deg, m)
  std::optional<Eigen::Vector3d> ac_attitude_rpy;

  PoseInput(const Eigen::Matrix<double, Eigen::Dynamic, 2> &points2d,
            const Eigen::Matrix<double, Eigen::Dynamic, 3> &points3d,
            const std::optional<Eigen::Vector3d> &attitude = std::nullopt)
      : points_2d(points2d), points_3d(points3d), ac_attitude_rpy(attitude) {
    if (points_2d.rows() != points_3d.rows()) {
      throw std::invalid_argument(
          "points_2d and points_3d must have the same number of points");
    }
  }
};

class Pose {
 public:
  void setup(const PoseSetup &pose_setup);
  std::optional<Eigen::Vector3d> compute_pose(
      const PoseInput &pose_input) const;
  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> solve_tpnp(
      const Eigen::Matrix<double, Eigen::Dynamic, 2> &points_2d,
      const Eigen::Matrix<double, Eigen::Dynamic, 3> &points_3d,
      const Eigen::Vector3d &ac_attitude_rpy) const;
  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> solve_opencv_pnp(
      const Eigen::Matrix<double, Eigen::Dynamic, 2> &points_2d,
      const Eigen::Matrix<double, Eigen::Dynamic, 3> &points_3d) const;

 private:
  PoseSetup pose_setup;

 private:
  Eigen::Vector3d cam_offset_to_runway_frame(
      const Eigen::Matrix3d &runway_to_camcv) const;
  Eigen::Matrix3d runway_to_camcv_frame(
      const Eigen::Vector3d &ac_attitude_rpy) const;
  Eigen::Vector3d lla_to_runway_frame(const Eigen::Vector3d &lla) const;
};
}  // namespace pose

#endif
