// Unit tests for the maneuver player logic (M1.7a): Maneuver (table check,
// zero-order hold) and ManeuverPlayer (start on the first clock, states).
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "asv_sim/maneuver.hpp"

namespace
{

using asv_sim::Maneuver;
using asv_sim::ManeuverPlayer;
using asv_sim::ManeuverRow;
using asv_sim::PlayerState;
using asv_sim::ThrusterLimits;

// The current values in config/asv.yaml (thrusters section) [REPO].
const ThrusterLimits kLimits{50.0, -40.0, 1.5708};

// Two command rows and the stop row: (0..10 s) straight, (10..20 s) turn.
Maneuver two_rows()
{
  return Maneuver({{0.0, 0.1, 0.0, 0.0}, {10.0, 0.1, 0.3, 0.05}, {20.0, 0.0, 0.0, 0.0}});
}

TEST(Maneuver, HoldsEachRowUntilTheNext)
{
  const auto m = two_rows();
  EXPECT_EQ(m.active_row(0.0), 0);
  EXPECT_EQ(m.active_row(9.999), 0);
  EXPECT_EQ(m.active_row(10.0), 1);  // the new row starts exactly at t_k
  EXPECT_EQ(m.active_row(19.999), 1);
  EXPECT_DOUBLE_EQ(m.command_at(5.0).throttle, 0.1);
  EXPECT_DOUBLE_EQ(m.command_at(5.0).steer, 0.0);
  EXPECT_DOUBLE_EQ(m.command_at(15.0).steer, 0.3);
  EXPECT_DOUBLE_EQ(m.command_at(15.0).spin, 0.05);
  EXPECT_DOUBLE_EQ(m.end_time(), 20.0);
  EXPECT_EQ(m.command_rows(), 2u);
}

TEST(Maneuver, ZeroBeforeStartAndAfterEnd)
{
  const auto m = two_rows();
  for (double tau : {-1.0, 20.0, 100.0}) {
    EXPECT_EQ(m.active_row(tau), -1) << tau;
    const auto c = m.command_at(tau);
    EXPECT_EQ(c.throttle, 0.0);
    EXPECT_EQ(c.steer, 0.0);
    EXPECT_EQ(c.spin, 0.0);
  }
}

TEST(Maneuver, RejectsBadTables)
{
  using Rows = std::vector<ManeuverRow>;
  // Fewer than two rows.
  EXPECT_THROW(Maneuver(Rows{{0.0, 0.0, 0.0, 0.0}}), std::invalid_argument);
  // First time not 0.
  EXPECT_THROW(Maneuver(Rows{{1.0, 0.1, 0.0, 0.0}, {2.0, 0.0, 0.0, 0.0}}),
    std::invalid_argument);
  // Times not strictly increasing.
  EXPECT_THROW(
    Maneuver(Rows{{0.0, 0.1, 0.0, 0.0}, {5.0, 0.1, 0.0, 0.0}, {5.0, 0.0, 0.0, 0.0}}),
    std::invalid_argument);
  // Last row not zero.
  EXPECT_THROW(Maneuver(Rows{{0.0, 0.1, 0.0, 0.0}, {5.0, 0.0, 0.1, 0.0}}),
    std::invalid_argument);
  // Not finite.
  EXPECT_THROW(Maneuver(Rows{{0.0, std::nan(""), 0.0, 0.0}, {5.0, 0.0, 0.0, 0.0}}),
    std::invalid_argument);
}

TEST(Maneuver, FromColumnsChecksLengths)
{
  const auto m = Maneuver::from_columns({0.0, 4.0}, {0.2, 0.0}, {0.0, 0.0}, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(m.command_at(1.0).throttle, 0.2);
  EXPECT_THROW(
    Maneuver::from_columns({0.0, 4.0}, {0.2}, {0.0, 0.0}, {0.0, 0.0}),
    std::invalid_argument);
}

TEST(ManeuverPlayer, WaitsForTheClockThenTheDelay)
{
  ManeuverPlayer p(two_rows(), 3.0, kLimits);
  // Clock still 0: no /clock message yet.
  auto out = p.step(0.0);
  EXPECT_EQ(out.state, PlayerState::WAITING);
  EXPECT_FALSE(p.started());
  // First time 2.0 s -> start at 5.0 s.
  out = p.step(2.0);
  EXPECT_EQ(out.state, PlayerState::WAITING);
  EXPECT_EQ(out.status, "WAITING");
  EXPECT_TRUE(p.started());
  EXPECT_DOUBLE_EQ(p.start_time(), 5.0);
  EXPECT_EQ(out.command.left_thrust, 0.0);
  EXPECT_EQ(p.step(4.99).state, PlayerState::WAITING);
}

TEST(ManeuverPlayer, RunsRowsThroughMixThenHoldsZero)
{
  ManeuverPlayer p(two_rows(), 3.0, kLimits);
  p.step(2.0);  // start at 5.0 s
  auto out = p.step(5.0);  // tau = 0
  EXPECT_EQ(out.state, PlayerState::RUNNING);
  EXPECT_EQ(out.status, "RUNNING 1/2");
  EXPECT_DOUBLE_EQ(out.command.left_thrust, 5.0);  // 0.1 * 50 N
  EXPECT_DOUBLE_EQ(out.command.right_thrust, 5.0);

  out = p.step(16.0);  // tau = 11: row 2, the same as mix(0.1, 0.3, 0.05)
  EXPECT_EQ(out.status, "RUNNING 2/2");
  const auto expected = asv_sim::mix(0.1, 0.3, 0.05, kLimits);
  EXPECT_DOUBLE_EQ(out.command.left_thrust, expected.left_thrust);
  EXPECT_DOUBLE_EQ(out.command.right_thrust, expected.right_thrust);
  EXPECT_DOUBLE_EQ(out.command.left_pos, expected.left_pos);
  EXPECT_DOUBLE_EQ(out.command.right_pos, -0.3);  // pod angle = -steer

  for (double t : {25.0, 26.0, 1000.0}) {  // tau >= t_N = 20: DONE, zero (P3)
    out = p.step(t);
    EXPECT_EQ(out.state, PlayerState::DONE) << t;
    EXPECT_EQ(out.status, "DONE");
    EXPECT_EQ(out.command.left_thrust, 0.0);
    EXPECT_EQ(out.command.right_thrust, 0.0);
    EXPECT_EQ(out.command.left_pos, 0.0);
  }
}

TEST(ManeuverPlayer, StartIsFixedByTheFirstClockOnly)
{
  ManeuverPlayer p(two_rows(), 0.0, kLimits);
  p.step(7.0);
  p.step(8.0);
  EXPECT_DOUBLE_EQ(p.start_time(), 7.0);
}

TEST(ManeuverPlayer, RejectsNegativeDelay)
{
  EXPECT_THROW(ManeuverPlayer(two_rows(), -1.0, kLimits), std::invalid_argument);
}

}  // namespace
