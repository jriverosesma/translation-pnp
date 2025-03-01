#include <Eigen/Dense>
#include <iostream>
#include <optional>

#include "TPnP/pose.hpp"

int main() {
  // Define pose estimator
  pose::Pose pose;

  // Define runway parameters
  double runway_true_heading = -60.9843658153;
  Eigen::Matrix<double, 4, 3> runway_keypoints_lla;
  Eigen::Vector3d runway_ltp_lla;
  runway_keypoints_lla << 45.59496878, -122.62161525, -15.08052988, 45.59532827,
      -122.62133041, -15.08052988, 45.58069445, -122.58375925, -15.06773041,
      45.58033505, -122.58404425, -15.06773041;
  runway_ltp_lla << 45.58051475, -122.58390175, -15.0677304067;

  // Define camera parameters
  Eigen::Matrix3d cam_intrinsic;
  Eigen::Vector<double, 5> cam_dist;
  Eigen::Vector3d cam_offset;
  Eigen::Vector3d cam_attitude_rpy;
  cam_intrinsic << 7.41050131e+03, 0, 2.04698374e+03, 0, 7.41050131e+03,
      1.50325490e+03, 0, 0, 1;
  cam_dist << 0.10571245, 0, 0, 0, 0;
  cam_offset << 0.12984, -0.02857, -0.918739606368141;
  cam_attitude_rpy << 0.02046669, -0.26113752, -0.76237039;

  // Define A/C parameters
  Eigen::Vector3d ac_attitude_rpy;
  ac_attitude_rpy << -2.13718960e-01, -7.03531966, 3.02248654e+02;

  // Define detected 2D points
  Eigen::Matrix<double, 4, 2> points_2d;  // Distorted 2D points
  points_2d << 1693.96928761, 674.34990729, 1772.79367443, 674.51209595,
      1917.81319812, 1075.57163956, 1574.07248558, 1075.79556393;

  // Define pose estimator setup parameters
  pose::PoseSetup pose_setup = {runway_true_heading, runway_ltp_lla,
                                cam_intrinsic,       cam_dist,
                                cam_offset,          cam_attitude_rpy};

  // Setup pose estimator
  pose.setup(pose_setup);

  // Define pose estimator input
  pose::PoseInput pose_input_attitude = {points_2d, runway_keypoints_lla,
                                         ac_attitude_rpy};
  pose::PoseInput pose_input_no_attitude = {points_2d, runway_keypoints_lla,
                                            std::nullopt};

  // Compute A/C position using TPnP
  std::optional<Eigen::Vector3d> xyz_tpnp =
      pose.compute_pose(pose_input_attitude);

  // Compute A/C position using OpenCV PnP
  std::optional<Eigen::Vector3d> xyz_opencvpnp =
      pose.compute_pose(pose_input_no_attitude);

  // Display results
  Eigen::Vector3d expected_xyz;
  expected_xyz << -986.509, 2.05654, 68.8754;

  std::cout << "[C++ EXAMPLE RESULTS]" << std::endl;
  std::cout << "GT: " << std::endl << expected_xyz << std::endl;
  std::cout << "TPnP: " << std::endl << xyz_tpnp.value() << std::endl;
  std::cout << "OpenCV PnP: " << std::endl
            << xyz_opencvpnp.value() << std::endl;
};
