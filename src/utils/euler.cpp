#include "TPnP/utils/euler.hpp"

#include <Eigen/Dense>
#include <cmath>

#include "TPnP/utils/math.hpp"

namespace euler {
Eigen::Matrix3d euler_to_matrix(double x, double y, double z, bool degrees) {
  if (degrees) {
    x = math::degrees_to_radians(x);
    y = math::degrees_to_radians(y);
    z = math::degrees_to_radians(z);
  }

  Eigen::Matrix3d rx, ry, rz;
  double cos_x = std::cos(x);
  double sin_x = std::sin(x);
  double cos_y = std::cos(y);
  double sin_y = std::sin(y);
  double cos_z = std::cos(z);
  double sin_z = std::sin(z);

  rx << 1.0, 0.0, 0.0, 0.0, cos_x, -sin_x, 0.0, sin_x, cos_x;

  ry << cos_y, 0.0, sin_y, 0.0, 1.0, 0.0, -sin_y, 0.0, cos_y;

  rz << cos_z, -sin_z, 0.0, sin_z, cos_z, 0.0, 0.0, 0.0, 1.0;

  // Rotation order = X -> Y -> Z
  return rz * ry * rx;
}

Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d &xyz, bool degrees) {
  return euler_to_matrix(xyz[0], xyz[1], xyz[2], degrees);
}
}  // namespace euler
