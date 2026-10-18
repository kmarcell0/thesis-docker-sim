#include "asv_sim/angles.hpp"

#include <cmath>

namespace asv_sim
{

double wrap_angle(double angle)
{
  // std::remainder gives the result in [-pi, pi]; fold -pi onto +pi.
  const double wrapped = std::remainder(angle, 2.0 * M_PI);
  return wrapped <= -M_PI ? M_PI : wrapped;
}

double yaw_from_quaternion(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace asv_sim
