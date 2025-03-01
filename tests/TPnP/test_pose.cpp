#include <Eigen/Dense>
#include <catch2/catch_all.hpp>
#include <optional>

#include "TPnP/pose.hpp"

class PoseFixture {
 public:
  pose::Pose pose;
  Eigen::Matrix<double, 4, 3> runway_keypoints_lla;
  Eigen::Vector3d ac_attitude_rpy;
  Eigen::Matrix<double, 4, 2> points_2d;  // Distorted 2D points
  Eigen::Matrix<double, 4, 3> points_3d;  // Runway reference frame

  PoseFixture() {
    // Test data taken from image with GUID = 25311217
    double runway_true_heading = -60.9843658153;
    Eigen::Vector3d runway_ltp_lla;
    Eigen::Matrix3d cam_intrinsic;
    Eigen::Vector<double, 5> cam_dist;
    Eigen::Vector3d cam_offset;
    Eigen::Vector3d cam_attitude_rpy;
    runway_ltp_lla << 45.58051475, -122.58390175, -15.0677304067;
    cam_intrinsic << 7.41050131e+03, 0, 2.04698374e+03, 0, 7.41050131e+03,
        1.50325490e+03, 0, 0, 1;
    cam_dist << 0.10571245, 0, 0, 0, 0;
    cam_offset << 0.12984, -0.02857, -0.918739606368141;
    cam_attitude_rpy << 0.02046669, -0.26113752, -0.76237039;

    pose::PoseSetup pose_setup = {runway_true_heading, runway_ltp_lla,
                                  cam_intrinsic,       cam_dist,
                                  cam_offset,          cam_attitude_rpy};
    pose.setup(pose_setup);

    runway_keypoints_lla << 45.59496878, -122.62161525, -15.08052988,
        45.59532827, -122.62133041, -15.08052988, 45.58069445, -122.58375925,
        -15.06773041, 45.58033505, -122.58404425, -15.06773041;
    ac_attitude_rpy << -2.13718960e-01, -7.03531966, 3.02248654e+02;
    points_2d << 1693.96928761, 674.34990729, 1772.79367443, 674.51209595,
        1917.81319812, 1075.57163956, 1574.07248558, 1075.79556393;
    points_3d << 3352.86, 21.8885, -0.893264, 3352.78, -23.8317, -0.893264,
        -0.03811, -22.8601, -4.10028e-05, 0.0381819, 22.8601, -4.1003e-05;
  }
};

TEST_CASE("pose_input", "[Different number of 2D and 3D points]") {
  Eigen::Matrix<double, 2, 2> points_2d;
  Eigen::Matrix<double, 1, 3> points_3d;
  points_2d << 0, 1, 2, 3;
  points_3d << 0, 1, 2;

  REQUIRE_THROWS_AS(pose::PoseInput(points_2d, points_3d),
                    std::invalid_argument);
}

TEST_CASE_METHOD(PoseFixture, "solve_tpnp", "[TPnP]") {
  Eigen::Matrix<double, 4, 2> undistorted_points_2d;
  undistorted_points_2d << 1694.52, 675.643, 1773.2, 675.726, 1917.86, 1075.74,
      1574.44, 1076.13;

  Eigen::Vector3d expected_xyz;
  Eigen::Matrix3d expected_runway_to_camcv;
  expected_xyz << -994.574, 2.56081, 69.6008;
  expected_runway_to_camcv << -0.0427965, -0.999082, 0.00171769, -0.126998,
      0.00373474, -0.991896, 0.990979, -0.0426678, -0.127042;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> computed_pose =
      pose.solve_tpnp(undistorted_points_2d, points_3d, ac_attitude_rpy);

  REQUIRE(computed_pose.has_value());
  REQUIRE(computed_pose.value().first.isApprox(expected_runway_to_camcv, 1e-3));
  REQUIRE(computed_pose.value().second.isApprox(expected_xyz, 1e-3));
}

TEST_CASE_METHOD(PoseFixture, "solve_tpnp", "[Not enough points]") {
  Eigen::Matrix<double, 1, 2> points_2d;
  Eigen::Matrix<double, 1, 3> points_3d;
  points_2d << 0, 1;
  points_3d << 0, 1, 2;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> computed_pose =
      pose.solve_tpnp(points_2d, points_3d, ac_attitude_rpy);

  REQUIRE(!computed_pose.has_value());
}

TEST_CASE_METHOD(PoseFixture, "solve_opencv_pnp") {
  Eigen::Vector3d expected_xyz;
  Eigen::Matrix3d expected_runway_to_camcv;
  expected_xyz << -987.429, 2.1815, 68.8792;
  expected_runway_to_camcv << 0.0426591, 0.126846, -0.991005, 0.999088,
      -0.00371542, 0.0425315, -0.00171294, 0.991916, 0.126888;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> computed_pose =
      pose.solve_opencv_pnp(points_2d, points_3d);

  REQUIRE(computed_pose.has_value());
  REQUIRE(computed_pose.value().first.isApprox(expected_runway_to_camcv, 1e-3));
  REQUIRE(computed_pose.value().second.isApprox(expected_xyz, 1e-3));
}

TEST_CASE_METHOD(PoseFixture, "solve_opencv_pnp", "[Not enough points]") {
  Eigen::Matrix<double, 2, 2> points_2d;
  Eigen::Matrix<double, 2, 3> points_3d;
  points_2d << 0, 1, 2, 3;
  points_3d << 0, 1, 2, 3, 4, 5;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> computed_pose =
      pose.solve_opencv_pnp(points_2d, points_3d);

  REQUIRE(!computed_pose.has_value());
}

TEST_CASE_METHOD(PoseFixture, "compute_pose", "[TPnP]") {
  Eigen::Vector3d expected_xyz;
  expected_xyz << -994.822, 2.54197, 68.7051;

  pose::PoseInput pose_input = {points_2d, runway_keypoints_lla,
                                ac_attitude_rpy};

  std::optional<Eigen::Vector3d> xyz = pose.compute_pose(pose_input);

  REQUIRE(xyz.has_value());
  REQUIRE(xyz->isApprox(expected_xyz, 1e-6));
}

TEST_CASE_METHOD(PoseFixture, "compute_pose", "[OpenCV PnP]") {
  Eigen::Vector3d expected_xyz;
  expected_xyz << -986.509, 2.05654, 68.8754;

  pose::PoseInput pose_input = {points_2d, runway_keypoints_lla, std::nullopt};

  std::optional<Eigen::Vector3d> xyz = pose.compute_pose(pose_input);

  REQUIRE(xyz.has_value());
  REQUIRE(xyz->isApprox(expected_xyz, 1e-3));
}
