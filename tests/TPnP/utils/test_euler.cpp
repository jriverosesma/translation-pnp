#include <Eigen/Dense>
#include <catch2/catch_all.hpp>

#include "TPnP/utils/euler.hpp"

TEST_CASE("euler_to_matrix") {
  Eigen::Vector3d xyz;
  xyz << -1.001488770299387, -8.33681633377578, -3.233396042357981;

  Eigen::Matrix3d expected_matrix;
  expected_matrix << 0.98785771, 0.05892503, -0.14375323, -0.05580743,
      0.9981126, 0.02562736, 0.14499201, -0.01729369, 0.98928168;

  Eigen::Matrix3d matrix = euler::euler_to_matrix(xyz);

  REQUIRE(matrix.isApprox(expected_matrix, 1e-6));
}
