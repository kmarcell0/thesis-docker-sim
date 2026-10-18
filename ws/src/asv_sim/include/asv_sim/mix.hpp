// Thrust allocation: (throttle, steer, spin) -> the four thruster commands.
// ROS-free, so it can be unit tested and shared by the waypoint follower,
// the maneuver player and the teleop. Spec: milestone1_plan.md section 5.5,
// decisions 10.1 and 10.2 (2026-10-18).
#pragma once

namespace asv_sim
{

// Actuator limits. Values come from config/asv.yaml (thrusters: max_thrust,
// min_thrust, steer_limit); never typed into code.
struct ThrusterLimits
{
  double max_thrust;   // N, forward limit of one thruster (T_max)
  double min_thrust;   // N, reverse limit of one thruster (T_min, negative)
  double steer_limit;  // rad, largest pod angle magnitude
};

struct ThrusterCommand
{
  double left_thrust;   // N (T_L)
  double right_thrust;  // N (T_R)
  double left_pos;      // rad, pod angle delta; positive turns the boat RIGHT
  double right_pos;     // rad, same value as left_pos
};

// throttle: mean thrust as a fraction of max_thrust (1 = full forward).
// steer:    turn command in rad; steer > 0 turns the boat LEFT (counter-
//           clockwise) while the thrust is forward. Pod angle delta = -steer.
// spin:     differential thrust as a fraction of max_thrust; spin > 0 turns
//           the boat LEFT (right thruster pushes more than the left).
//
// Wanted thrusts: T_L* = T_max (throttle - spin), T_R* = T_max (throttle + spin).
// On saturation the difference T_R - T_L (the turn) is kept and the mean
// thrust gives way ("turning wins", decision 10.1 b).
ThrusterCommand mix(double throttle, double steer, double spin, const ThrusterLimits & limits);

}  // namespace asv_sim
