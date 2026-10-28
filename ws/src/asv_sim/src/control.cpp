#include "asv_sim/control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "asv_core/angles.hpp"

namespace asv_sim
{

namespace
{

// True for x >= 0; false for negative values and NaN.
bool non_negative(double x) {return x >= 0.0;}

}  // namespace

double course_angle(double heading, double u, double v, double min_speed)
{
  if (!non_negative(min_speed)) {
    throw std::invalid_argument("course_angle: min_speed must be >= 0");
  }
  // max() also covers reverse (u < 0): the course stays within +-90 deg of
  // the heading instead of flipping by pi.
  return asv_core::wrap_angle(heading + std::atan2(v, std::max(u, min_speed)));
}

HeadingControl::HeadingControl(const HeadingGains & gains)
: gains_(gains)
{
  if (!non_negative(gains_.heading_kp) || !non_negative(gains_.heading_kd) ||
    !non_negative(gains_.spin_threshold) || !non_negative(gains_.spin_kp) ||
    !non_negative(gains_.spin_max) || !non_negative(gains_.spin_small_kp))
  {
    throw std::invalid_argument("HeadingControl: gains and limits must be >= 0");
  }
  if (!(gains_.max_steer > 0.0)) {
    throw std::invalid_argument("HeadingControl: max_steer must be > 0");
  }
}

HeadingCommand HeadingControl::compute(
  double desired_heading, double heading, double yaw_rate) const
{
  HeadingCommand cmd;
  cmd.heading_error = asv_core::wrap_angle(desired_heading - heading);

  // PD law; the D part uses the measured yaw rate omega instead of the
  // derivative of psi~, so a jump of psi_d at a waypoint switch gives no
  // spike (milestone1_plan.md 5.3).
  cmd.steer = std::clamp(
    gains_.heading_kp * cmd.heading_error - gains_.heading_kd * yaw_rate,
    -gains_.max_steer, gains_.max_steer);

  // Spin: a ramp that starts at zero at psi_spin (10.3 b), plus a
  // proportional part at every error size (10.16 b); the sum is limited.
  const double excess = std::abs(cmd.heading_error) - gains_.spin_threshold;
  const double ramp = std::clamp(gains_.spin_kp * excess, 0.0, gains_.spin_max);
  cmd.spin = std::clamp(
    gains_.spin_small_kp * cmd.heading_error + std::copysign(ramp, cmd.heading_error),
    -gains_.spin_max, gains_.spin_max);
  return cmd;
}

SpeedControl::SpeedControl(const SpeedGains & gains)
: gains_(gains)
{
  if (!non_negative(gains_.drag_linear) || !non_negative(gains_.drag_quadratic) ||
    !non_negative(gains_.speed_kp) || !non_negative(gains_.speed_ki) ||
    !non_negative(gains_.stop_gain) || !non_negative(gains_.turn_threshold))
  {
    throw std::invalid_argument("SpeedControl: gains and coefficients must be >= 0");
  }
  if (!(gains_.max_thrust > 0.0)) {
    throw std::invalid_argument("SpeedControl: max_thrust must be > 0");
  }
  if (!(gains_.min_throttle <= 0.0) || !(gains_.max_throttle > 0.0)) {
    throw std::invalid_argument("SpeedControl: need min_throttle <= 0 < max_throttle");
  }
}

double SpeedControl::commanded_speed(double u_ref, double heading_error, double d_end) const
{
  // Slow down when pointing the wrong way and near the end of the route
  // (milestone1_plan.md 5.4). d_end < 0 (just past the end) must not ask for
  // a reverse speed, hence the outer max.
  const double heading_limited = u_ref * std::max(0.0, std::cos(heading_error));
  return std::max(0.0, std::min(heading_limited, gains_.stop_gain * d_end));
}

double SpeedControl::feedforward_throttle(double u) const
{
  // Steady state: thrust = drag, X_u u + X_uu u|u| (asv.yaml hydrodynamics),
  // shared by two thrusters of T_max each.
  const double thrust = gains_.drag_linear * u + gains_.drag_quadratic * u * std::abs(u);
  return thrust / (2.0 * gains_.max_thrust);
}

double SpeedControl::update(double u_cmd, double u, double heading_error, double dt)
{
  // 10.2 c: while turning hard the throttle may not go negative, because
  // with reverse thrust the pods turn the boat the wrong way.
  const double lower =
    std::abs(heading_error) > gains_.turn_threshold ? 0.0 : gains_.min_throttle;
  const double upper = gains_.max_throttle;

  const double error = u_cmd - u;
  const double base = feedforward_throttle(u_cmd) + gains_.speed_kp * error;

  // Try the step with the new integral; keep the old one if the output is
  // beyond a limit and the error pushes further that way (anti-windup by
  // conditional integration).
  double integral = integral_;
  if (dt > 0.0) {
    integral += error * dt;
  }
  double raw = base + gains_.speed_ki * integral;
  if ((raw > upper && error > 0.0) || (raw < lower && error < 0.0)) {
    integral = integral_;
    raw = base + gains_.speed_ki * integral;
  }
  integral_ = integral;
  return std::clamp(raw, lower, upper);
}

}  // namespace asv_sim
