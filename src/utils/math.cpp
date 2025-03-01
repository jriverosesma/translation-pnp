#include "TPnP/utils/math.hpp"

namespace math {
double radians_to_degrees(double angle_rad) { return angle_rad * 180.0 / PI; }

double degrees_to_radians(double angle_deg) { return angle_deg * PI / 180.0; }
}  // namespace math
