#include "asv_sim/mix.hpp"

#include <algorithm>

namespace asv_sim
{

ThrusterCommand mix(double throttle, double steer, double spin, const ThrusterLimits & limits)
{
  double left = limits.max_thrust * (throttle - spin);
  double right = limits.max_thrust * (throttle + spin);

  // Shift both sides by the same amount so the difference (the turning
  // moment 0.33 (T_R - T_L) cos delta, milestone1_plan.md 10.2) survives.
  // Only if the difference itself is wider than the range [T_min, T_max]
  // does the final clamp below cut it.
  const double high = std::max(left, right);
  const double low = std::min(left, right);
  if (high > limits.max_thrust) {
    const double shift = high - limits.max_thrust;
    left -= shift;
    right -= shift;
  } else if (low < limits.min_thrust) {
    const double shift = limits.min_thrust - low;
    left += shift;
    right += shift;
  }

  ThrusterCommand cmd;
  cmd.left_thrust = std::clamp(left, limits.min_thrust, limits.max_thrust);
  cmd.right_thrust = std::clamp(right, limits.min_thrust, limits.max_thrust);
  // Pods sit behind the centre of gravity: a negative pod angle turns the
  // boat left (confirmed in Gazebo, Step 1), so steer > 0 -> delta < 0.
  cmd.left_pos = std::clamp(-steer, -limits.steer_limit, limits.steer_limit);
  cmd.right_pos = cmd.left_pos;
  return cmd;
}

}  // namespace asv_sim
