// The Milestone 1 waypoint follower without ROS: one control step of the
// whole chain (Route -> LosGuidance -> HeadingControl, SpeedControl -> mix())
// plus the state machine of milestone1_plan.md section 5.7 and the gate
// monitor. The node (src/waypoint_follower.cpp) only converts messages; the
// closed-loop test drives the 3-DOF model through the same class.
// Decisions S1 a, S2 a, S3 a of the M1.3 brief (2026-10-20).
// Symbols as in docs/glossary.md.
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "asv_sim/control.hpp"
#include "asv_sim/gate_monitor.hpp"
#include "asv_sim/guidance.hpp"
#include "asv_sim/mix.hpp"

namespace asv_sim
{

// Everything the follower needs. Route, gains, buoys and timings come from
// config/task1_route.yaml; the boat values (drag, thrust and steering limits,
// footprint) from config/asv.yaml.
struct FollowerConfig
{
  std::vector<Point2> waypoints;  // W_0 ... W_{n_w}, world frame, m
  std::vector<double> speeds;     // u_ref per segment, m/s (n_w values)
  double switch_distance;         // R_switch, m
  double lookahead;               // Delta, m
  HeadingGains heading;
  double course_min_speed;        // u_min, m/s (course_angle())
  SpeedGains speed;
  ThrusterLimits limits;
  std::vector<Gate> gates;
  std::vector<Buoy> other_buoys;  // clearance check only
  BoatFootprint footprint;
  double gate_margin;             // d_m, m
  double gate_capture;            // m
  double start_delay;             // s after the first odometry, >= 0
  double max_run_time;            // s from the start of RUNNING, > 0
  double odom_timeout;            // s, largest odometry age, > 0
};

// The boat state taken from one odometry message.
struct NavState
{
  double stamp;    // s, header.stamp (simulation time)
  Point2 p;        // m, world frame
  double heading;  // psi, rad, world frame
  double u;        // m/s, forward speed (body frame)
  double v;        // m/s, sideways speed (body frame, > 0 = to the left)
  double yaw_rate; // omega, rad/s
};

enum class FollowerState
{
  WAITING,   // no odometry yet, or the start delay is running
  RUNNING,
  FINISHED,  // route done; zero thrust, pods straight; latched (S3 a)
  STOPPED    // odometry too old or run too long; zero thrust; latched (S3 a)
};

struct FollowerOutput
{
  FollowerState state;
  std::string status;        // WAITING, RUNNING i/n_w, FINISHED, STOPPED: <reason>
  ThrusterCommand command;   // all zero unless RUNNING
  bool has_target = false;   // true only while RUNNING
  Point2 target{0.0, 0.0};   // p_LOS, m
  double cross_track = 0.0;  // e, m (valid while RUNNING)
  std::vector<GateEvent> events;  // gate crossings found in this step
};

class Follower
{
public:
  // Throws std::invalid_argument for a wrong number of speeds, a negative
  // start_delay, max_run_time <= 0 or odom_timeout <= 0, and passes on the
  // checks of Route, LosGuidance, the controllers and GateMonitor.
  explicit Follower(const FollowerConfig & config);

  // One control step at time t (s, simulation time) with the latest odometry
  // (std::nullopt before the first message). Order (section 5.7):
  //   FINISHED / STOPPED stay as they are (latched, S3 a)
  //   no odometry                     -> WAITING
  //   t - stamp > odom_timeout        -> STOPPED: odom timeout
  //   t - t_first < start_delay       -> WAITING   (t_first: first stamp seen)
  //   t - t_run > max_run_time        -> STOPPED: time limit (t_run: RUNNING start)
  //   route finished                  -> FINISHED
  //   else RUNNING: the control chain of test_closed_loop (M1.2e).
  // The gate monitor sees every new odometry message (also after FINISHED,
  // while the boat coasts). dt for the speed integral = difference of the
  // stamps used in consecutive RUNNING steps; 0 for the same message twice.
  FollowerOutput step(double t, const std::optional<NavState> & nav);

  FollowerState state() const {return state_;}
  const Route & route() const {return route_;}
  const GateMonitor & monitor() const {return monitor_;}

private:
  FollowerOutput hold(const std::string & status) const;  // zero command
  std::string running_status() const;

  FollowerConfig config_;
  Route route_;
  LosGuidance los_;
  HeadingControl heading_;
  SpeedControl speed_;
  GateMonitor monitor_;
  FollowerState state_ = FollowerState::WAITING;
  std::string stop_reason_;
  std::optional<double> first_stamp_;    // t_first
  std::optional<double> run_start_;      // t_run
  std::optional<double> control_stamp_;  // stamp used in the last RUNNING step
  std::optional<double> monitor_stamp_;  // stamp last given to the monitor
};

// One line for the status topic and the log, e.g.
// "GATE gate1 OUTBOUND offset +0.012 m PASS".
std::string gate_event_text(const GateEvent & event);

}  // namespace asv_sim
