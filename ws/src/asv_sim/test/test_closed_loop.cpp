// Closed-loop logic test (M1.2e): the full Milestone 1 control chain
// (Route -> LosGuidance -> HeadingControl, SpeedControl -> mix()) drives the
// 3-DOF boat model of boat_model.hpp along the Task 1 route, and GateMonitor
// judges the run. No Gazebo, no ROS. Since M1.3 the chain runs through
// asv_sim::Follower, the same code as the waypoint_follower node (S2 a).
// Boat values: config/asv.yaml and config/asv_report.json [REPO], read at test
// time. Route, gains and buoys: milestone1_plan.md sections 2 and 6 [EST
// starting gains, REPO geometry], the same values as config/task1_route.yaml
// (typed in here: the test does not read the ROS parameter file).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "asv_sim/follower.hpp"
#include "boat_model.hpp"

namespace
{

using asv_sim::GateDirection;
using asv_sim::GateEvent;
using asv_sim::GateResult;
using asv_sim::Point2;
using asv_sim_test::BoatModel;
using asv_sim_test::BoatParams;
using asv_sim_test::BoatState;

constexpr double kControlDt = 0.05;  // s, the follower's 20 Hz loop
constexpr int kSubSteps = 20;        // model steps per control step (2.5 ms)
constexpr double kMaxRunTime = 180.0;  // s, max_run_time (plan section 6)
constexpr double kCoastTime = 2.0;   // s with zero thrust after FINISHED

const BoatParams & params()
{
  static const BoatParams p = asv_sim_test::load_boat_params(ASV_CONFIG_DIR);
  return p;
}

// Run the model with one constant command for `duration` seconds.
BoatState hold(const asv_sim::ThrusterCommand & cmd, double duration)
{
  BoatModel boat(params());
  const double h = kControlDt / kSubSteps;
  for (double t = 0.0; t < duration; t += h) {
    boat.step(cmd, h);
  }
  return boat.state();
}

// Positive root of a w + b w^2 = c (steady state of linear + quadratic drag).
double steady(double a, double b, double c)
{
  return (-a + std::sqrt(a * a + 4.0 * b * c)) / (2.0 * b);
}

// ---- The model against its own equations and Step 1 [MEASURED] -----------

TEST(BoatModel, ParametersLoaded)
{
  const auto & p = params();
  EXPECT_NEAR(p.mass, 18.191, 1e-3);  // report total_kg
  EXPECT_GT(p.inertia_z, 2.80801);    // more than base_link alone (pods)
  EXPECT_NEAR(p.pod_x, -0.70, 1e-9);
  EXPECT_NEAR(p.pod_y, 0.33, 1e-9);
  EXPECT_NEAR(p.half_width, 0.44, 1e-9);
}

TEST(BoatModel, SteadySpeedMatchesDragAndStep1)
{
  // 10 N total, pods straight. Step 1: 1.00 m/s.
  const auto s = hold({5.0, 5.0, 0.0, 0.0}, 60.0);
  const auto & p = params();
  EXPECT_NEAR(s.u, steady(p.drag_u, p.drag_uu, 10.0), 1e-4);
  EXPECT_NEAR(s.u, 1.00, 0.10);
  EXPECT_NEAR(s.v, 0.0, 1e-9);
  EXPECT_NEAR(s.omega, 0.0, 1e-9);
}

TEST(BoatModel, PodTurnMatchesYawBalanceAndStep1)
{
  // 20 N total, pods at -0.4 rad. Step 1: +0.15 rad/s (turns LEFT).
  const auto s = hold({10.0, 10.0, -0.4, -0.4}, 60.0);
  const auto & p = params();
  const double moment = p.pod_x * 20.0 * std::sin(-0.4);  // N, > 0
  EXPECT_GT(s.omega, 0.0);
  EXPECT_NEAR(s.omega, steady(p.drag_r, p.drag_rr, moment), 1e-4);
  EXPECT_NEAR(s.omega, 0.15, 0.015);
  std::printf("[pod turn] u = %.3f m/s, v = %.3f m/s, omega = %.4f rad/s\n",
    s.u, s.v, s.omega);
}

TEST(BoatModel, DifferentialTurnMatchesYawBalanceAndStep1)
{
  // left -10 N, right +10 N, pods straight. Step 1: +0.18 rad/s.
  const auto s = hold({-10.0, 10.0, 0.0, 0.0}, 60.0);
  const auto & p = params();
  EXPECT_NEAR(s.omega, steady(p.drag_r, p.drag_rr, p.pod_y * 20.0), 1e-4);
  EXPECT_NEAR(s.omega, 0.18, 0.018);
  EXPECT_NEAR(s.u, 0.0, 1e-9);
}

TEST(BoatModel, PodAngleIsRateLimited)
{
  BoatModel boat(params());
  boat.step({0.0, 0.0, 1.0, 1.0}, 0.1);
  EXPECT_NEAR(boat.state().pod, params().steer_rate * 0.1, 1e-12);
}

// ---- Closed loop on the Task 1 route ---------------------------------------

struct RunResult
{
  std::vector<GateEvent> events;
  bool finished = false;
  bool collision = false;
  double time = 0.0;           // s until FINISHED (or the time limit)
  double max_cross_track = 0.0;  // max |e|, m
  double max_sway = 0.0;       // max |v|, m/s
  asv_sim::BuoyClearance closest;
};

RunResult run_task1(const BoatState & start)
{
  const auto & bp = params();
  asv_sim::FollowerConfig c;
  // Route W0 (the spawn, fixed in the parameter file: 10.15 a) ... W9
  // (plan section 2).
  c.waypoints = {{0.0, 0.0},
    {17.0, 0.0}, {22.0, 2.5}, {25.0, 1.0}, {25.5, -2.0}, {23.0, -4.0},
    {20.0, -2.5}, {17.0, -0.5}, {15.0, 0.0}, {-1.0, 0.0}};
  c.speeds = {1.0, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 1.0};
  c.switch_distance = 1.5;
  c.lookahead = 3.0;
  // heading_kp, heading_kd, max_steer, spin_threshold, spin_kp, spin_max,
  // spin_small_kp (spin_threshold 0.3, spin_kp 2.0: 10.14 a; spin_small_kp
  // 2.0: 10.16 b)
  c.heading = {1.2, 0.6, 0.8, 0.3, 2.0, 0.4, 2.0};
  c.course_min_speed = 0.3;  // m/s, u_min (M1.2f S1) [EST]
  // drag, max_thrust (asv.yaml); speed_kp, speed_ki, stop_gain,
  // turn_threshold, min_throttle, max_throttle (plan section 6)
  c.speed = {bp.drag_u, bp.drag_uu, bp.limits.max_thrust, 0.3, 0.05, 0.5, 0.3, -0.3, 1.0};
  c.limits = bp.limits;

  // Buoys (plan section 6) [REPO course and models/rb26_*/model.sdf].
  const double sm = 0.2286;
  const asv_sim::Buoy g1r{"gate1_red", {5.0, 1.524}, sm};
  const asv_sim::Buoy g1g{"gate1_green", {5.0, -1.524}, sm};
  const asv_sim::Buoy g2r{"gate2_red", {13.0, 1.524}, sm};
  const asv_sim::Buoy g2g{"gate2_green", {13.0, -1.524}, sm};
  c.gates = {{"gate1", g1r, g1g}, {"gate2", g2r, g2g}};
  c.other_buoys = {
    {"t2_ch1_red", {20.781, 6.0}, 0.1015}, {"t2_ch1_green", {23.219, 6.0}, 0.1015},
    {"t3_red", {30.0, -4.476}, 0.184}, {"t3_green", {30.0, -7.524}, 0.184}};
  c.footprint = {bp.hull_length, bp.half_width};
  c.gate_margin = 0.5;
  c.gate_capture = 2.0;
  // The model boat needs no time to settle; the loop below ends at
  // kMaxRunTime, so the follower's own time limit never fires here.
  c.start_delay = 0.0;
  c.max_run_time = kMaxRunTime;
  c.odom_timeout = 0.5;
  asv_sim::Follower follower(c);  // the node's code (M1.3, S2 a)

  BoatModel boat(bp, start);
  RunResult r;
  const double h = kControlDt / kSubSteps;
  double coast_left = kCoastTime;
  for (double t = 0.0; t < kMaxRunTime && coast_left > 0.0; t += kControlDt) {
    const auto & s = boat.state();
    // The model's state stands in for one odometry message stamped at t.
    const auto out = follower.step(t, asv_sim::NavState{t, {s.x, s.y}, s.psi, s.u, s.v, s.omega});
    r.events.insert(r.events.end(), out.events.begin(), out.events.end());
    if (out.state == asv_sim::FollowerState::RUNNING) {
      r.max_cross_track = std::max(r.max_cross_track, std::abs(out.cross_track));
    }
    if (out.state == asv_sim::FollowerState::FINISHED) {
      if (!r.finished) {
        r.finished = true;
        r.time = t;
      }
      coast_left -= kControlDt;
    }
    for (int k = 0; k < kSubSteps; ++k) {
      boat.step(out.command, h);
    }
    r.max_sway = std::max(r.max_sway, std::abs(boat.state().v));
  }
  if (!r.finished) {
    r.time = kMaxRunTime;
  }
  r.collision = follower.monitor().collision();
  r.closest = follower.monitor().closest();
  return r;
}

const char * name(GateResult g)
{
  switch (g) {
    case GateResult::PASS: return "PASS";
    case GateResult::FAIL_CLOSE: return "FAIL_CLOSE";
    default: return "FAIL_OUTSIDE";
  }
}

void expect_task1_done(const std::string & label, const BoatState & start)
{
  const RunResult r = run_task1(start);
  std::printf("[%s] finished %d at %.2f s, max |e| %.3f m, max |v| %.3f m/s, "
    "closest %s %.3f m, collision %d\n", label.c_str(), r.finished, r.time,
    r.max_cross_track, r.max_sway, r.closest.name.c_str(), r.closest.min_clearance,
    r.collision);
  for (const auto & e : r.events) {
    std::printf("[%s]   %s %s offset %+.3f m %s\n", label.c_str(), e.gate.c_str(),
      e.direction == GateDirection::OUTBOUND ? "OUT" : "IN", e.offset, name(e.result));
  }
  EXPECT_TRUE(r.finished) << label;
  EXPECT_FALSE(r.collision) << label;
  ASSERT_EQ(r.events.size(), 4u) << label;
  const char * gates[] = {"gate1", "gate2", "gate2", "gate1"};
  const GateDirection dirs[] = {GateDirection::OUTBOUND, GateDirection::OUTBOUND,
    GateDirection::INBOUND, GateDirection::INBOUND};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(r.events[i].gate, gates[i]) << label << " event " << i;
    EXPECT_EQ(r.events[i].direction, dirs[i]) << label << " event " << i;
    EXPECT_EQ(r.events[i].result, GateResult::PASS) << label << " event " << i;
  }
}

TEST(ClosedLoop, Task1FromSpawn)
{
  expect_task1_done("spawn", {});
}

TEST(ClosedLoop, Task1FromStartOffsets)
{
  BoatState s;
  s.y = 0.5;
  expect_task1_done("y+0.5", s);
  s.y = -0.5;
  expect_task1_done("y-0.5", s);
  s.y = 0.0;
  s.psi = 0.3;
  expect_task1_done("psi+0.3", s);
  s.psi = -0.3;
  expect_task1_done("psi-0.3", s);
}

}  // namespace
