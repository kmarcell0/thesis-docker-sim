// Keyboard teleop logic (M1.8): one key press -> new (throttle, steer, spin)
// levels. ROS-free, so it can be unit tested; the node (teleop_node.cpp) only
// reads keys, calls apply_key() and publishes mix() of the levels.
// A terminal sends no key-release events, so the levels stay set until
// another key changes them.
#pragma once

#include "asv_sim/mix.hpp"

namespace asv_sim
{

// The three inputs of mix(), with the same units and signs (mix.hpp).
struct TeleopLevels
{
  double throttle = 0.0;  // fraction of max_thrust
  double steer = 0.0;     // rad, > 0 turns LEFT
  double spin = 0.0;      // fraction of max_thrust, > 0 turns LEFT
};

// Change per key press. Values from config/teleop.yaml.
struct TeleopSteps
{
  double throttle;  // fraction of max_thrust
  double steer;     // rad
  double spin;      // fraction of max_thrust
};

struct TeleopUpdate
{
  TeleopLevels levels;
  bool quit = false;  // X or Ctrl-C: the node publishes zero and exits
};

// W/S throttle up/down, A/D steer left/right, Q/E spin left/right (either
// case), Space all zero, X or Ctrl-C (byte 3) quit; any other key changes
// nothing. The levels are clamped to throttle in [min_thrust / max_thrust, 1],
// steer in [-steer_limit, steer_limit], spin in [-1, 1]; mix() then handles
// thrust saturation.
TeleopUpdate apply_key(
  const TeleopLevels & levels, char key, const TeleopSteps & steps,
  const ThrusterLimits & limits);

}  // namespace asv_sim
