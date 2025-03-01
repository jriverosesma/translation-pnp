#ifndef EULER_H
#define EULER_H

#include <Eigen/Dense>

namespace euler {
Eigen::Matrix3d euler_to_matrix(double x, double y, double z,
                                bool degrees = true);

Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d &xyz,
                                bool degrees = true);
}  // namespace euler

#endif
