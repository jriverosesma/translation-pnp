#include "TPnP/utils/lla.hpp"

#include <Eigen/Dense>
#include <cmath>

#include "TPnP/utils/math.hpp"

constexpr double a = 6378137.0;            // WGS-84 Earth semimajor axis (m)
constexpr double f = 1.0 / 298.257223563;  // WGS-84 flattening
constexpr double e2 = f * (2 - f);         // Square of eccentricity

namespace lla {
// Convert LLA to ECEF (Earth-Centered Earth-Fixed)
Eigen::Vector3d lla_to_ecef(double lat, double lon, double alt, bool degrees) {
  if (degrees) {
    lat = math::degrees_to_radians(lat);
    lon = math::degrees_to_radians(lon);
  }

  double sin_lat = std::sin(lat);
  double cos_lat = std::cos(lat);
  double sin_lon = std::sin(lon);
  double cos_lon = std::cos(lon);

  double N = a / std::sqrt(1 - e2 * sin_lat * sin_lat);  // Radius of curvature

  double x = (N + alt) * cos_lat * cos_lon;
  double y = (N + alt) * cos_lat * sin_lon;
  double z = (N * (1 - e2) + alt) * sin_lat;

  return Eigen::Vector3d(x, y, z);
}

// Convert ECEF to NED using reference LLA
Eigen::Vector3d ecef_to_ned(const Eigen::Vector3d &ecef,
                            const Eigen::Vector3d &ecef_ref, double lat_ref,
                            double lon_ref, bool degrees) {
  if (degrees) {
    lat_ref = math::degrees_to_radians(lat_ref);
    lon_ref = math::degrees_to_radians(lon_ref);
  }

  Eigen::Matrix3d R;
  double sin_lat = std::sin(lat_ref);
  double cos_lat = std::cos(lat_ref);
  double sin_lon = std::sin(lon_ref);
  double cos_lon = std::cos(lon_ref);

  R(0, 0) = -sin_lat * cos_lon;
  R(0, 1) = -sin_lat * sin_lon;
  R(0, 2) = cos_lat;
  R(1, 0) = -sin_lon;
  R(1, 1) = cos_lon;
  R(1, 2) = 0.0;
  R(2, 0) = -cos_lat * cos_lon;
  R(2, 1) = -cos_lat * sin_lon;
  R(2, 2) = -sin_lat;

  Eigen::Vector3d delta_ecef = ecef - ecef_ref;
  return R * delta_ecef;
}

// Convert LLA to NED relative to reference LLA
Eigen::Vector3d lla_to_ned(double lat, double lon, double alt, double lat_ref,
                           double lon_ref, double alt_ref, bool degrees) {
  Eigen::Vector3d ecef = lla_to_ecef(lat, lon, alt, degrees);
  Eigen::Vector3d ecef_ref = lla_to_ecef(lat_ref, lon_ref, alt_ref, degrees);
  return ecef_to_ned(ecef, ecef_ref, lat_ref, lon_ref, degrees);
}

Eigen::Vector3d lla_to_ned(const Eigen::Vector3d lla,
                           const Eigen::Vector3d lla_ref, bool degrees) {
  return lla_to_ned(lla[0], lla[1], lla[2], lla_ref[0], lla_ref[1], lla_ref[2],
                    degrees);
}
}  // namespace lla
