#ifndef LLA_H
#define LLA_H

#include <Eigen/Dense>

namespace lla {
Eigen::Vector3d lla_to_ned(double lat, double lon, double alt, double lat_ref,
                           double lon_ref, double alt_ref, bool degrees = true);

Eigen::Vector3d lla_to_ned(const Eigen::Vector3d lla,
                           const Eigen::Vector3d lla_ref, bool degrees = true);
}  // namespace lla

#endif
