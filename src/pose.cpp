#include "TPnP/pose.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "TPnP/utils/euler.hpp"
#include "TPnP/utils/lla.hpp"

namespace pose {
void Pose::setup(const PoseSetup &pose_setup) { this->pose_setup = pose_setup; }

std::optional<Eigen::Vector3d> Pose::compute_pose(
    const PoseInput &pose_input) const {
  // 1. Transform 3D points from LLA to runway reference frame
  int n_points = pose_input.points_2d.rows();

  Eigen::Matrix<double, Eigen::Dynamic, 3> points_3d(n_points, 3);

  for (int i = 0; i < n_points; i++) {
    points_3d.row(i) = this->lla_to_runway_frame(pose_input.points_3d.row(i));
  }
  // 2. Compute camera position in runway reference frame
  // TPnP case
  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> pose;
  if (pose_input.ac_attitude_rpy.has_value()) {
    // Undistort 2D points before computing position using TPnP
    cv::Mat cv_points_2d(n_points, 1, CV_64FC2);
    cv::Mat cv_cam_intrinsic(3, 3, CV_64F);
    cv::Mat cv_cam_dist(5, 1, CV_64F);
    cv::Mat cv_points_2d_undistort(n_points, 1, CV_64FC2);
    Eigen::Matrix<double, Eigen::Dynamic, 2> points_2d_undistort(n_points, 2);

    cv::eigen2cv(pose_input.points_2d, cv_points_2d);
    cv::eigen2cv(pose_setup.cam_intrinsic, cv_cam_intrinsic);
    cv::eigen2cv(pose_setup.cam_dist, cv_cam_dist);

    cv::undistortImagePoints(cv_points_2d, cv_points_2d_undistort,
                             cv_cam_intrinsic, cv_cam_dist);

    // The following fails: cv::eigen2cv(cv_points_2d_undistort,
    // points_2d_undistort)
    for (int i = 0; i < n_points; ++i) {
      cv::Vec2d &point = cv_points_2d_undistort.at<cv::Vec2d>(i, 0);
      points_2d_undistort(i, 0) = point[0];
      points_2d_undistort(i, 1) = point[1];
    }

    pose = this->solve_tpnp(points_2d_undistort, points_3d,
                            *pose_input.ac_attitude_rpy);
  }
  // OpenCVPnP case
  else {
    pose = this->solve_opencv_pnp(pose_input.points_2d, points_3d);
  }
  if (!pose.has_value()) {
    return std::nullopt;
  }

  // 3. Compute camera position offset w.r.t. A/C INS position
  Eigen::Vector3d cam_offset =
      this->cam_offset_to_runway_frame(pose.value().first);

  // 4. Substract camera position offset from camera position to get A/C INS
  // position
  return pose.value().second - cam_offset;
}

std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> Pose::solve_tpnp(
    const Eigen::Matrix<double, Eigen::Dynamic, 2> &points_2d,
    const Eigen::Matrix<double, Eigen::Dynamic, 3> &points_3d,
    const Eigen::Vector3d &ac_attitude_rpy) const {
  int n_points = points_2d.rows();

  if (n_points < 2) {
    std::cerr << "[ERROR] Must pass 2 or more points for TPnP to work"
              << std::endl;
    return std::nullopt;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> a(2 * n_points, 3);
  Eigen::Vector<double, Eigen::Dynamic> b(2 * n_points);
  Eigen::Matrix<double, Eigen::Dynamic, 3> points_3d_camcv(n_points, 3);
  double fx = this->pose_setup.cam_intrinsic(0, 0);
  double fy = this->pose_setup.cam_intrinsic(1, 1);
  double cx = this->pose_setup.cam_intrinsic(0, 2);
  double cy = this->pose_setup.cam_intrinsic(1, 2);
  Eigen::Matrix3d runway_to_camcv =
      Pose::runway_to_camcv_frame(ac_attitude_rpy);
  for (int i = 0; i < n_points; i++) {
    points_3d_camcv.row(i) = runway_to_camcv * points_3d.row(i).transpose();
    a.row(2 * i) << fx, 0, cx - points_2d(i, 0);
    a.row(2 * i + 1) << 0, fy, cy - points_2d(i, 1);
    b.block<2, 1>(2 * i, 0) =
        a.block<2, 3>(2 * i, 0) * points_3d_camcv.row(i).transpose();
  }

  a *= runway_to_camcv;

  Eigen::Vector3d tvec =
      a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  return std::make_pair(runway_to_camcv, tvec);
}

std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
Pose::solve_opencv_pnp(
    const Eigen::Matrix<double, Eigen::Dynamic, 2> &points_2d,
    const Eigen::Matrix<double, Eigen::Dynamic, 3> &points_3d) const {
  int n_points = points_2d.rows();

  if (n_points < 4) {
    std::cerr << "[ERROR] Must pass 4 or more points for OpenCV PnP to work"
              << std::endl;
    return std::nullopt;
  }

  cv::Mat cv_points_3d(n_points, 3, CV_64F);
  cv::Mat cv_points_2d(n_points, 2, CV_64F);
  cv::Mat cv_cam_intrinsic(3, 3, CV_64F);
  cv::Mat cv_cam_dist(5, 1, CV_64F);
  cv::Mat cv_rvec(3, 1, CV_64F);
  cv::Mat cv_tvec(3, 1, CV_64F);
  bool useExtrinsicGuess = false;
  int flags = cv::SOLVEPNP_ITERATIVE;

  cv::eigen2cv(points_3d, cv_points_3d);
  cv::eigen2cv(points_2d, cv_points_2d);
  cv::eigen2cv(pose_setup.cam_intrinsic, cv_cam_intrinsic);
  cv::eigen2cv(pose_setup.cam_dist, cv_cam_dist);

  bool success =
      cv::solvePnP(cv_points_3d, cv_points_2d, cv_cam_intrinsic, cv_cam_dist,
                   cv_rvec, cv_tvec, useExtrinsicGuess, flags);

  if (success) {
    cv::Mat cv_rot;
    Eigen::Matrix3d runway_to_camcv;
    Eigen::Vector3d tvec;

    cv::Rodrigues(cv_rvec, cv_rot);
    cv_tvec = -cv_rot.t() * cv_tvec;

    cv::cv2eigen(cv_tvec, tvec);
    cv::cv2eigen(-cv_rot.t(), runway_to_camcv);

    return std::make_pair(runway_to_camcv, tvec);
  } else {
    std::clog << "[WARNING] OpenCV PnP did not converge" << std::endl;
    return std::nullopt;
  }
}

Eigen::Vector3d Pose::cam_offset_to_runway_frame(
    const Eigen::Matrix3d &runway_to_camcv) const {
  Eigen::Matrix3d ac_to_cam =
      euler::euler_to_matrix(this->pose_setup.cam_attitude_rpy).transpose();
  Eigen::Matrix3d cam_to_camcv;
  cam_to_camcv << 0, 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix3d ac_to_runway =
      runway_to_camcv.inverse() * cam_to_camcv * ac_to_cam;

  return ac_to_runway * this->pose_setup.cam_offset;
}

Eigen::Matrix3d Pose::runway_to_camcv_frame(
    const Eigen::Vector3d &ac_attitude_rpy) const {
  Eigen::Matrix3d runway_to_ned =
      euler::euler_to_matrix(180, 0, this->pose_setup.runway_true_heading)
          .transpose();
  Eigen::Matrix3d ned_to_ac =
      euler::euler_to_matrix(ac_attitude_rpy).transpose();
  Eigen::Matrix3d ac_to_cam =
      euler::euler_to_matrix(this->pose_setup.cam_attitude_rpy).transpose();
  Eigen::Matrix3d cam_to_camcv;
  cam_to_camcv << 0, 1, 0, 0, 0, 1, 1, 0, 0;

  return cam_to_camcv * ac_to_cam * ned_to_ac * runway_to_ned;
}

Eigen::Vector3d Pose::lla_to_runway_frame(const Eigen::Vector3d &lla) const {
  Eigen::Matrix3d ned_to_runway =
      euler::euler_to_matrix(180, 0, this->pose_setup.runway_true_heading);

  Eigen::Vector3d xyz_ned =
      lla::lla_to_ned(lla, this->pose_setup.runway_ltp_lla);

  return ned_to_runway * xyz_ned;
}
}  // namespace pose
