// Unit tests for asv_sim::mix() (M1.2a).
#include <gtest/gtest.h>

#include <cmath>

#include "asv_sim/mix.hpp"

namespace
{

using asv_sim::mix;
using asv_sim::ThrusterCommand;
using asv_sim::ThrusterLimits;

// The current values in config/asv.yaml (thrusters section) [REPO].
const ThrusterLimits kLimits{50.0, -40.0, 1.5708};

// Yaw moment of the two pods (milestone1_plan.md 10.2), N m; positive = left
// turn. Pods 0.70 m behind base_link (thrusters.x) and 0.33 m to each side
// (hull.spacing / 2) [REPO asv.yaml]. Only its sign is used in the tests.
double yaw_moment(const ThrusterCommand & c)
{
  const double delta = c.left_pos;
  return -0.70 * (c.left_thrust + c.right_thrust) * std::sin(delta) +
         0.33 * (c.right_thrust - c.left_thrust) * std::cos(delta);
}

constexpr double kTol = 1e-12;

TEST(Mix, ZeroInZeroOut)
{
  const auto c = mix(0.0, 0.0, 0.0, kLimits);
  EXPECT_NEAR(c.left_thrust, 0.0, kTol);
  EXPECT_NEAR(c.right_thrust, 0.0, kTol);
  EXPECT_NEAR(c.left_pos, 0.0, kTol);
  EXPECT_NEAR(c.right_pos, 0.0, kTol);
}

TEST(Mix, ThrottleIsFractionOfMaxThrust)
{
  const auto c = mix(0.5, 0.0, 0.0, kLimits);
  EXPECT_NEAR(c.left_thrust, 25.0, kTol);
  EXPECT_NEAR(c.right_thrust, 25.0, kTol);
  EXPECT_NEAR(c.left_pos, 0.0, kTol);
}

TEST(Mix, PositiveSteerGivesNegativePodAngleAndLeftTurn)
{
  const auto c = mix(0.2, 0.3, 0.0, kLimits);
  EXPECT_NEAR(c.left_pos, -0.3, kTol);
  EXPECT_NEAR(c.right_pos, -0.3, kTol);
  EXPECT_GT(yaw_moment(c), 0.0);
}

TEST(Mix, PositiveSpinGivesLeftTurn)
{
  const auto c = mix(0.2, 0.0, 0.1, kLimits);
  EXPECT_NEAR(c.left_thrust, 5.0, kTol);
  EXPECT_NEAR(c.right_thrust, 15.0, kTol);
  EXPECT_LT(c.left_thrust, c.right_thrust);
  EXPECT_GT(yaw_moment(c), 0.0);
}

TEST(Mix, ThrustClampedToLimits)
{
  const auto fwd = mix(2.0, 0.0, 0.0, kLimits);
  EXPECT_NEAR(fwd.left_thrust, 50.0, kTol);
  EXPECT_NEAR(fwd.right_thrust, 50.0, kTol);
  const auto rev = mix(-2.0, 0.0, 0.0, kLimits);
  EXPECT_NEAR(rev.left_thrust, -40.0, kTol);
  EXPECT_NEAR(rev.right_thrust, -40.0, kTol);
}

TEST(Mix, PodAngleClampedToSteerLimit)
{
  EXPECT_NEAR(mix(0.0, 3.0, 0.0, kLimits).left_pos, -1.5708, kTol);
  EXPECT_NEAR(mix(0.0, -3.0, 0.0, kLimits).right_pos, 1.5708, kTol);
}

// Decision 10.1 b: on saturation the left/right difference is kept.
TEST(Mix, SaturationAtFullThrottleKeepsDifference)
{
  const auto c = mix(1.0, 0.0, 0.4, kLimits);  // wanted (30, 70) N
  EXPECT_NEAR(c.left_thrust, 10.0, kTol);
  EXPECT_NEAR(c.right_thrust, 50.0, kTol);
}

TEST(Mix, SaturationInReverseKeepsDifference)
{
  const auto c = mix(-1.0, 0.0, 0.2, kLimits);  // wanted (-60, -40) N
  EXPECT_NEAR(c.left_thrust, -40.0, kTol);
  EXPECT_NEAR(c.right_thrust, -20.0, kTol);
}

// A difference wider than T_max - T_min = 90 N can't be kept: both sides clip.
TEST(Mix, DifferenceWiderThanRangeClipsBothSides)
{
  const auto c = mix(0.0, 0.0, 1.0, kLimits);  // wanted (-50, 50) N
  EXPECT_NEAR(c.left_thrust, -40.0, kTol);
  EXPECT_NEAR(c.right_thrust, 50.0, kTol);
  const auto d = mix(0.0, 0.0, -1.0, kLimits);  // wanted (50, -50) N
  EXPECT_NEAR(d.left_thrust, 50.0, kTol);
  EXPECT_NEAR(d.right_thrust, -40.0, kTol);
}

}  // namespace
