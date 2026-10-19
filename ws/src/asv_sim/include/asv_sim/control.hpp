// Heading and speed control for the Milestone 1 waypoint follower: turn the
// desired heading and speed into the (throttle, steer, spin) that mix() takes.
// ROS-free. Spec: milestone1_plan.md sections 5.3 and 5.4, decisions 10.2 c
// and 10.3 b (2026-10-18); course feedback 10.13 a and proportional spin
// 10.16 a (2026-10-19, M1.2f).
// Symbols as in docs/glossary.md.
#pragma once

namespace asv_sim
{

// Course chi: the direction the boat moves (milestone1_plan.md 5.3,
// decision 10.13 a, course-based LOS after Fossen). heading psi (rad, world
// frame); u, v: forward and sideways speed (m/s, body frame, v > 0 = to the
// left); min_speed u_min (m/s, >= 0):
//   chi = wrap(psi + atan2(v, max(u, u_min)))
// The max() is the low-speed guard (M1.2f S1): near u = 0 the sideslip
// atan2(v, u) is meaningless and could swing to +-90 deg. Pass chi instead of
// psi to HeadingControl::compute(). Throws std::invalid_argument for
// min_speed < 0 or NaN.
double course_angle(double heading, double u, double v, double min_speed);

// Gains of HeadingControl. Starting values in config/task1_route.yaml
// (milestone1_plan.md section 6), tuned in M1.5.
struct HeadingGains
{
  double heading_kp;      // K_psi, rad steer per rad heading error, >= 0
  double heading_kd;      // K_omega, rad steer per rad/s yaw rate, >= 0
  double max_steer;       // rad, steer limit, > 0
  double spin_threshold;  // psi_spin, rad, spin assist starts above it, >= 0
  double spin_kp;         // K_spin, spin per rad of error above psi_spin, >= 0
  double spin_max;        // spin limit (fraction of max_thrust), >= 0
  double spin_small_kp;   // K_s, spin per rad of error at any size, >= 0 (10.16 a)
};

struct HeadingCommand
{
  double heading_error;  // psi~ = wrap(psi_d - psi), rad (> 0: turn left)
  double steer;          // rad, > 0 turns LEFT (mix() convention)
  double spin;           // fraction of max_thrust, > 0 turns LEFT
};

// PD heading control with the derivative on the measured yaw rate, plus spin
// (differential thrust): proportional at every error size (10.16 a), with a
// stronger ramp for large errors.
class HeadingControl
{
public:
  // Throws std::invalid_argument for a negative gain or max_steer <= 0.
  explicit HeadingControl(const HeadingGains & gains);

  // desired_heading psi_d, heading psi (rad, world frame), yaw_rate omega
  // (rad/s, body frame, > 0 counter-clockwise).
  //   psi~  = wrap(psi_d - psi)
  //   steer = clamp(K_psi psi~ - K_omega omega, +-max_steer)          (5.3)
  //   spin  = clamp(K_s psi~
  //                 + sign(psi~) clamp(K_spin (|psi~| - psi_spin), 0, spin_max),
  //                 +-spin_max)
  //   (10.3 b: the ramp starts at the threshold, so spin has no jump;
  //   10.16 a: K_s psi~ turns the boat without the sideways push of the aft
  //   pods, which damps the cross-track swing after an offset start)
  // Stateless.
  HeadingCommand compute(double desired_heading, double heading, double yaw_rate) const;

private:
  HeadingGains gains_;
};

// Parameters of SpeedControl. The drag coefficients and max_thrust come from
// config/asv.yaml (hydrodynamics xU, xUU; thrusters max_thrust); the gains
// and limits from config/task1_route.yaml.
struct SpeedGains
{
  double drag_linear;     // X_u, N per m/s, >= 0
  double drag_quadratic;  // X_uu, N per (m/s)^2, >= 0
  double max_thrust;      // T_max, N, one thruster, > 0
  double speed_kp;        // K_P, throttle per m/s, >= 0
  double speed_ki;        // K_I, throttle per m, >= 0
  double stop_gain;       // K_stop, 1/s, >= 0
  double turn_threshold;  // psi_turn, rad: above it throttle >= 0 (10.2 c), >= 0
  double min_throttle;    // lower throttle limit when not turning, <= 0
  double max_throttle;    // upper throttle limit, > 0
};

// Speed control: drag-model feedforward plus a PI correction with
// anti-windup. Holds the integral, so one object per run.
class SpeedControl
{
public:
  // Throws std::invalid_argument for a negative gain or coefficient,
  // max_thrust <= 0, min_throttle > 0 or max_throttle <= 0.
  explicit SpeedControl(const SpeedGains & gains);

  // u_cmd = max(0, min(u_ref max(0, cos psi~), K_stop d_end))      (5.4)
  // u_ref: the segment's speed, m/s; heading_error psi~, rad; d_end: distance
  // left along the route, m (Route::remaining_distance()).
  double commanded_speed(double u_ref, double heading_error, double d_end) const;

  // T_ff(u) / (2 T_max) with T_ff(u) = X_u u + X_uu u|u|: the throttle that
  // holds speed u against the drag model.
  double feedforward_throttle(double u) const;

  // One control step. u_cmd from commanded_speed(), u: measured forward
  // speed (m/s, body frame), dt: time since the last step from the message
  // stamps (s; dt <= 0 skips integration).
  //   throttle = clamp(T_ff(u_cmd)/(2 T_max) + K_P (u_cmd - u) + K_I I,
  //                    lower, max_throttle)
  //   lower = 0 while |psi~| > psi_turn (10.2 c: the pods keep pushing
  //   forward, so steer keeps its sign), else min_throttle.
  // Anti-windup: I += (u_cmd - u) dt only if that does not push the output
  // further into the limit it is at.
  double update(double u_cmd, double u, double heading_error, double dt);

  double integral() const {return integral_;}  // I, m
  void reset() {integral_ = 0.0;}

private:
  SpeedGains gains_;
  double integral_ = 0.0;
};

}  // namespace asv_sim
