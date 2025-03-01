#include <Eigen/Dense>
#include <catch2/catch_all.hpp>

#include "TPnP/utils/lla.hpp"

TEST_CASE("lla_to_ned") {
  // Define the target and reference LLA (latitude, longitude, altitude)
  // coordinates in degrees and meters
  Eigen::Vector3d lla;
  lla << 45.976, 7.658, 4531.0;

  Eigen::Vector3d lla_ref;
  lla_ref << 46.017, 7.750, 1673.0;

  Eigen::Vector3d expected_ned;
  expected_ned << -4556.3215138458918, -7134.7571959798834, -2852.3904239449948;

  // Convert LLA to NED
  Eigen::Vector3d ned = lla::lla_to_ned(lla, lla_ref);

  REQUIRE(ned.isApprox(expected_ned, 1e-6));
}
