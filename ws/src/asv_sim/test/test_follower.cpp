// Unit tests for Follower (M1.3): the state machine of milestone1_plan.md
// section 5.7 and that RUNNING gives exactly the M1.2 control chain.
// Route and gate are a small made-up course; gains and timings as in plan
// section 6 [EST]; boat limits and drag as in config/asv.yaml [REPO].
#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>

#include "asv_sim/follower.hpp"

namespace
{

using asv_sim::Follower;
using asv_sim::FollowerConfig;
using asv_sim::FollowerOutput;
using asv_sim::FollowerState;
using asv_sim::NavState;
using asv_sim::Point2;

constexpr double kSurmark = 0.2286;  // m, models/rb26_surmark_*/model.sdf

FollowerConfig config()
{
  FollowerConfig c;
  c.waypoints = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}};
  c.speeds = {1.0, 0.6};
  c.switch_distance = 1.5;
  c.lookahead = 3.0;
  c.heading = {1.2, 0.6, 0.8, 0.3, 2.0, 0.4, 2.0};
  c.course_min_speed = 0.3;
  c.speed = {4.0, 6.0, 50.0, 0.3, 0.05, 0.5, 0.3, -0.3, 1.0};
  c.limits = {50.0, -40.0, 1.5708};
  c.gates = {{"gate1", {"gate1_red", {5.0, 1.524}, kSurmark},
    {"gate1_green", {5.0, -1.524}, kSurmark}}};
  c.other_buoys = {};
  c.footprint = {1.60, 0.44};
  c.gate_margin = 0.5;
  c.gate_capture = 2.0;
  c.start_delay = 3.0;
  c.max_run_time = 10.0;
  c.odom_timeout = 0.5;
  return c;
}

// A boat at (x, y), heading east, moving slowly forward.
NavState nav(double stamp, double x, double y = 0.0, double u = 0.2)
{
  return NavState{stamp, {x, y}, 0.0, u, 0.0, 0.0};
}

void expect_zero(const FollowerOutput & out)
{
  EXPECT_EQ(out.command.left_thrust, 0.0);
  EXPECT_EQ(out.command.right_thrust, 0.0);
  EXPECT_EQ(out.command.left_pos, 0.0);
  EXPECT_EQ(out.command.right_pos, 0.0);
  EXPECT_FALSE(out.has_target);
}

TEST(Follower, WaitsWithoutOdometry)
{
  Follower f(config());
  const auto out = f.step(5.0, std::nullopt);
  EXPECT_EQ(out.state, FollowerState::WAITING);
  EXPECT_EQ(out.status, "WAITING");
  expect_zero(out);
}

TEST(Follower, WaitsForStartDelayThenRuns)
{
  Follower f(config());
  for (double t = 1.0; t < 3.99; t += 0.5) {  // first message at 1.0 s
    const auto out = f.step(t, nav(t, 0.0));
    EXPECT_EQ(out.status, "WAITING") << t;
    expect_zero(out);
  }
  const auto out = f.step(4.0, nav(4.0, 0.0));  // t - t_first = 3.0 = start_delay
  EXPECT_EQ(out.state, FollowerState::RUNNING);
  EXPECT_EQ(out.status, "RUNNING 1/2");
  EXPECT_TRUE(out.has_target);
  EXPECT_GT(out.command.left_thrust, 0.0);
  EXPECT_GT(out.command.right_thrust, 0.0);
}

TEST(Follower, StopsOnOldOdometryAndStaysStopped)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  EXPECT_EQ(f.step(0.0, nav(0.0, 0.0)).state, FollowerState::RUNNING);
  // Age 0.5 s is still allowed (the rule is age > odom_timeout).
  EXPECT_EQ(f.step(0.5, nav(0.0, 0.0)).state, FollowerState::RUNNING);
  const auto out = f.step(0.55, nav(0.0, 0.0));
  EXPECT_EQ(out.state, FollowerState::STOPPED);
  EXPECT_EQ(out.status, "STOPPED: odom timeout");
  expect_zero(out);
  // Latched (S3 a): fresh odometry does not restart the run.
  const auto later = f.step(0.6, nav(0.6, 0.0));
  EXPECT_EQ(later.status, "STOPPED: odom timeout");
  expect_zero(later);
}

TEST(Follower, StopsOnOldOdometryWhileWaiting)
{
  Follower f(config());
  f.step(0.0, nav(0.0, 0.0));
  EXPECT_EQ(f.step(1.0, nav(0.0, 0.0)).status, "STOPPED: odom timeout");
}

TEST(Follower, StopsAtTimeLimit)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  EXPECT_EQ(f.step(2.0, nav(2.0, 0.0)).state, FollowerState::RUNNING);  // t_run = 2
  EXPECT_EQ(f.step(12.0, nav(12.0, 1.0)).state, FollowerState::RUNNING);  // 10 s: allowed
  const auto out = f.step(12.05, nav(12.05, 1.0));
  EXPECT_EQ(out.status, "STOPPED: time limit");
  expect_zero(out);
  EXPECT_EQ(f.step(12.1, nav(12.1, 1.0)).status, "STOPPED: time limit");
}

TEST(Follower, FinishesWithZeroCommandAndStaysFinished)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  EXPECT_EQ(f.step(0.0, nav(0.0, 0.0)).status, "RUNNING 1/2");
  EXPECT_EQ(f.step(0.1, nav(0.1, 19.0)).status, "RUNNING 2/2");  // switch at l - 1.5
  const auto out = f.step(0.2, nav(0.2, 20.0, 9.0));            // last switch rule
  EXPECT_EQ(out.state, FollowerState::FINISHED);
  EXPECT_EQ(out.status, "FINISHED");
  expect_zero(out);
  const auto later = f.step(0.3, nav(0.3, 0.0));  // back at the start: still done
  EXPECT_EQ(later.status, "FINISHED");
  expect_zero(later);
}

TEST(Follower, RunningGivesTheM12ControlChain)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  const NavState n{0.0, {3.0, 0.4}, 0.1, 0.8, 0.05, 0.02};
  const auto out = f.step(0.0, n);

  // The same chain by hand (test_closed_loop before M1.3), first step dt = 0.
  asv_sim::Route route(c.waypoints, c.switch_distance);
  route.update(n.p);
  const auto target = asv_sim::LosGuidance(c.lookahead).compute(route, n.p);
  const double chi = asv_sim::course_angle(n.heading, n.u, n.v, c.course_min_speed);
  const auto hc = asv_sim::HeadingControl(c.heading).compute(
    target.desired_heading, chi, n.yaw_rate);
  asv_sim::SpeedControl speed(c.speed);
  const double u_cmd = speed.commanded_speed(
    c.speeds[0], hc.heading_error, route.remaining_distance());
  const auto cmd = asv_sim::mix(
    speed.update(u_cmd, n.u, hc.heading_error, 0.0), hc.steer, hc.spin, c.limits);

  EXPECT_DOUBLE_EQ(out.command.left_thrust, cmd.left_thrust);
  EXPECT_DOUBLE_EQ(out.command.right_thrust, cmd.right_thrust);
  EXPECT_DOUBLE_EQ(out.command.left_pos, cmd.left_pos);
  EXPECT_DOUBLE_EQ(out.command.right_pos, cmd.right_pos);
  EXPECT_DOUBLE_EQ(out.target.x, target.target.x);
  EXPECT_DOUBLE_EQ(out.target.y, target.target.y);
  EXPECT_DOUBLE_EQ(out.cross_track, route.cross_track());
}

TEST(Follower, IntegratesWithStampDifferences)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  asv_sim::SpeedControl ref(c.speed);
  // Irregular stamps; the follower must use 0, 0.07 and 0.03 s as dt.
  const double stamps[] = {0.0, 0.07, 0.10};
  const double dts[] = {0.0, 0.07, 0.03};
  for (int k = 0; k < 3; ++k) {
    const auto out = f.step(stamps[k], nav(stamps[k], 1.0));
    // Heading error is 0 on the centre line, so throttle = mean thrust / T_max.
    const double u_cmd = ref.commanded_speed(1.0, 0.0, 20.0 - 1.0 + 10.0);
    const double throttle = ref.update(u_cmd, 0.2, 0.0, dts[k]);
    EXPECT_NEAR(out.command.left_thrust, throttle * c.limits.max_thrust, 1e-12) << k;
  }
  EXPECT_GT(ref.integral(), 0.0);
}

TEST(Follower, SameMessageTwiceDoesNotIntegrate)
{
  auto c = config();
  c.start_delay = 0.0;
  Follower f(c);
  f.step(0.0, nav(0.0, 1.0));
  const auto a = f.step(0.05, nav(0.05, 1.0));
  const auto b = f.step(0.10, nav(0.05, 1.0));  // no new odometry
  EXPECT_EQ(a.command.left_thrust, b.command.left_thrust);
  const auto d = f.step(0.15, nav(0.15, 1.0));  // new message: dt = 0.10 s
  EXPECT_GT(d.command.left_thrust, b.command.left_thrust);
}

TEST(Follower, ReportsGateCrossingsAlsoWhileWaiting)
{
  Follower f(config());  // start delay 3 s: stays WAITING here
  EXPECT_TRUE(f.step(0.0, nav(0.0, 4.9, 0.01)).events.empty());
  const auto out = f.step(0.05, nav(0.05, 5.1, 0.01));
  EXPECT_EQ(out.status, "WAITING");
  ASSERT_EQ(out.events.size(), 1u);
  EXPECT_EQ(asv_sim::gate_event_text(out.events[0]),
    "GATE gate1 OUTBOUND offset +0.010 m PASS");
  // The same message again is not a new pose.
  EXPECT_TRUE(f.step(0.1, nav(0.05, 5.1, 0.01)).events.empty());
}

TEST(Follower, RejectsBadConfig)
{
  auto c = config();
  c.speeds = {1.0};
  EXPECT_THROW(Follower{c}, std::invalid_argument);
  c = config();
  c.start_delay = -1.0;
  EXPECT_THROW(Follower{c}, std::invalid_argument);
  c = config();
  c.max_run_time = 0.0;
  EXPECT_THROW(Follower{c}, std::invalid_argument);
  c = config();
  c.odom_timeout = 0.0;
  EXPECT_THROW(Follower{c}, std::invalid_argument);
  c = config();
  c.lookahead = 0.0;  // passed on from LosGuidance
  EXPECT_THROW(Follower{c}, std::invalid_argument);
}

}  // namespace
