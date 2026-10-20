// Maneuver player logic (M1.7a): a timed table of (throttle, steer, spin)
// played open loop (no feedback) with a zero-order hold between rows.
// ROS-free, so it can be unit tested; the maneuver_player node only does I/O.
// Spec: milestone1_plan.md, M1.7a brief (decisions P2 hold, P3 zero after
// the end, 2026-10-20); timing: docs/design.md section 3 rule 5.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "asv_sim/mix.hpp"

namespace asv_sim
{

// One row of the table: from `time` on, until the next row's time, the
// command is (throttle, steer, spin), with the meaning of mix() (mix.hpp).
struct ManeuverRow
{
  double time;      // t_k, s since the maneuver start (tau)
  double throttle;  // fraction of max_thrust
  double steer;     // rad, > 0 turns LEFT
  double spin;      // fraction of max_thrust, > 0 turns LEFT
};

class Maneuver
{
public:
  // Checks the table and throws std::invalid_argument naming the problem:
  // at least two rows; t_0 = 0; times strictly increasing; all values
  // finite; the last row all zero (it marks the end, t_N).
  explicit Maneuver(std::vector<ManeuverRow> rows);

  // Builds the rows from four equally long columns (the YAML layout).
  static Maneuver from_columns(
    const std::vector<double> & times, const std::vector<double> & throttle,
    const std::vector<double> & steer, const std::vector<double> & spin);

  // Zero-order hold: row k for t_k <= tau < t_{k+1}. Returns -1 before the
  // start (tau < 0) and after the end (tau >= t_N).
  int active_row(double tau) const;

  // c(tau): the active row's command, zero outside [0, t_N).
  ManeuverRow command_at(double tau) const;

  double end_time() const {return rows_.back().time;}  // t_N, s
  // N: the rows that command something (all but the final stop row).
  std::size_t command_rows() const {return rows_.size() - 1;}

private:
  std::vector<ManeuverRow> rows_;
};

enum class PlayerState
{
  WAITING,  // no simulation time yet, or inside start_delay
  RUNNING,
  DONE      // after t_N; zero thrust from now on (P3)
};

struct PlayerOutput
{
  PlayerState state;
  std::string status;       // "WAITING", "RUNNING k/N" (k from 1), "DONE"
  ThrusterCommand command;  // mix() of c(tau); zero unless RUNNING
};

// The maneuver on the ROS clock. The first nonzero time t seen sets the
// start t_start = t + start_delay (with use_sim_time the clock reads 0 until
// the first /clock message); then tau = t - t_start.
class ManeuverPlayer
{
public:
  // start_delay: s, >= 0 (throws std::invalid_argument otherwise).
  ManeuverPlayer(Maneuver maneuver, double start_delay, const ThrusterLimits & limits);

  // t: current ROS time, s.
  PlayerOutput step(double t);

  bool started() const {return has_start_;}
  double start_time() const {return start_time_;}  // t_start, s (valid once started())

private:
  Maneuver maneuver_;
  double start_delay_;
  ThrusterLimits limits_;
  bool has_start_ = false;
  double start_time_ = 0.0;
};

}  // namespace asv_sim
