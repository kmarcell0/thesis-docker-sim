// Angle helpers: wrapping and yaw from a quaternion. ROS-free.
// Every angle difference in the project goes through wrap_angle() (design.md
// section 2); when asv_core exists (M2.1) this helper moves there.
#pragma once

namespace asv_sim
{

// Wraps an angle to (-pi, pi], in rad. -pi maps to +pi, so each direction
// has exactly one value.
double wrap_angle(double angle);

// Heading psi (yaw, rad, counter-clockwise from east in ENU) of the unit
// quaternion (w, x, y, z), for the Z-Y-X (yaw-pitch-roll) convention of ROS:
//   psi = atan2(2 (w z + x y), 1 - 2 (y^2 + z^2))   (milestone1_plan.md 5)
// Roll and pitch do not change the result.
double yaw_from_quaternion(double w, double x, double y, double z);

}  // namespace asv_sim
