// Unit tests for HeadingControl and SpeedControl (M1.2c), with the starting
// values of milestone1_plan.md section 6 [EST] and the boat values of
// config/asv.yaml [REPO].
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "asv_sim/control.hpp"

namespace
{

using asv_sim::HeadingControl;
using asv_sim::HeadingGains;
using asv_sim::SpeedControl;
using asv_sim::SpeedGains;

constexpr double kTol = 1e-9;
constexpr double kDt = 0.05;  // s, one step at 20 Hz

// heading_kp, heading_kd, max_steer, spin_threshold, spin_kp, spin_max
const HeadingGains kHeading{1.2, 0.6, 0.8, 0.8, 1.0, 0.4};

// xU, xUU, max_thrust (asv.yaml); speed_kp, speed_ki, stop_gain,
// turn_threshold, min_throttle, max_throttle (task1_route.yaml)
const SpeedGains kSpeed{4.0, 6.0, 50.0, 0.3, 0.05, 0.5, 0.3, -0.3, 1.0};

// ---- HeadingControl -------------------------------------------------------

TEST(HeadingControl, SteerSignAndProportionalTerm)
{
  HeadingControl hc(kHeading);
  const auto left = hc.compute(0.2, 0.0, 0.0);
  EXPECT_NEAR(left.heading_error, 0.2, kTol);
  EXPECT_NEAR(left.steer, 1.2 * 0.2, kTol);  // > 0: turn left
  const auto right = hc.compute(-0.2, 0.0, 0.0);
  EXPECT_NEAR(right.steer, -1.2 * 0.2, kTol);
}

TEST(HeadingControl, SteerClampedToMaxSteer)
{
  HeadingControl hc(kHeading);
  EXPECT_NEAR(hc.compute(1.0, 0.0, 0.0).steer, 0.8, kTol);
  EXPECT_NEAR(hc.compute(-1.0, 0.0, 0.0).steer, -0.8, kTol);
}

TEST(HeadingControl, YawRateDamping)
{
  HeadingControl hc(kHeading);
  // On course but turning left at 0.5 rad/s: steer right to stop the turn.
  EXPECT_NEAR(hc.compute(0.0, 0.0, 0.5).steer, -0.6 * 0.5, kTol);
}

TEST(HeadingControl, ErrorWrappedAcrossPi)
{
  HeadingControl hc(kHeading);
  // psi_d = 3.1, psi = -3.1: the short way is 0.083 rad to the RIGHT, not
  // 6.2 rad to the left.
  const auto cmd = hc.compute(3.1, -3.1, 0.0);
  EXPECT_NEAR(cmd.heading_error, 6.2 - 2.0 * M_PI, kTol);
  EXPECT_LT(cmd.steer, 0.0);
  EXPECT_NEAR(cmd.spin, 0.0, kTol);
}

TEST(HeadingControl, SpinZeroUpToThreshold)
{
  HeadingControl hc(kHeading);
  EXPECT_EQ(hc.compute(0.0, 0.0, 0.0).spin, 0.0);
  EXPECT_EQ(hc.compute(0.5, 0.0, 0.0).spin, 0.0);
  EXPECT_EQ(hc.compute(0.8, 0.0, 0.0).spin, 0.0);
  EXPECT_EQ(hc.compute(-0.8, 0.0, 0.0).spin, 0.0);
}

TEST(HeadingControl, SpinContinuousAtThreshold)
{
  HeadingControl hc(kHeading);
  // 10.3 b: just above psi_spin the spin is tiny, not a jump to spin_max.
  EXPECT_LT(std::abs(hc.compute(0.8 + 1e-6, 0.0, 0.0).spin), 1e-5);
  EXPECT_LT(std::abs(hc.compute(-0.8 - 1e-6, 0.0, 0.0).spin), 1e-5);
}

TEST(HeadingControl, SpinRampAndSaturation)
{
  HeadingControl hc(kHeading);
  EXPECT_NEAR(hc.compute(1.0, 0.0, 0.0).spin, 0.2, kTol);  // 1.0 * (1.0 - 0.8)
  EXPECT_NEAR(hc.compute(1.2, 0.0, 0.0).spin, 0.4, kTol);  // reaches spin_max
  EXPECT_NEAR(hc.compute(2.0, 0.0, 0.0).spin, 0.4, kTol);  // stays there
}

TEST(HeadingControl, SpinOddInError)
{
  HeadingControl hc(kHeading);
  for (double err : {0.9, 1.1, 1.5, 3.0}) {
    EXPECT_NEAR(hc.compute(-err, 0.0, 0.0).spin, -hc.compute(err, 0.0, 0.0).spin, kTol);
  }
}

TEST(HeadingControl, InvalidGainsThrow)
{
  HeadingGains g = kHeading;
  g.max_steer = 0.0;
  EXPECT_THROW(HeadingControl{g}, std::invalid_argument);
  g = kHeading;
  g.heading_kp = -1.0;
  EXPECT_THROW(HeadingControl{g}, std::invalid_argument);
  g = kHeading;
  g.spin_max = -0.1;
  EXPECT_THROW(HeadingControl{g}, std::invalid_argument);
  g = kHeading;
  g.spin_threshold = NAN;
  EXPECT_THROW(HeadingControl{g}, std::invalid_argument);
}

// ---- SpeedControl ---------------------------------------------------------

TEST(SpeedControl, CommandedSpeedCosineFactor)
{
  SpeedControl sc(kSpeed);
  EXPECT_NEAR(sc.commanded_speed(0.6, 0.0, 50.0), 0.6, kTol);
  EXPECT_NEAR(sc.commanded_speed(0.6, 1.0, 50.0), 0.6 * std::cos(1.0), kTol);
  // Pointing sideways or backwards: no forward speed wanted.
  EXPECT_EQ(sc.commanded_speed(0.6, M_PI / 2.0 + 0.1, 50.0), 0.0);
  EXPECT_EQ(sc.commanded_speed(0.6, M_PI, 50.0), 0.0);
}

TEST(SpeedControl, CommandedSpeedCappedNearEnd)
{
  SpeedControl sc(kSpeed);
  EXPECT_NEAR(sc.commanded_speed(1.0, 0.0, 1.0), 0.5, kTol);  // K_stop d_end
  EXPECT_NEAR(sc.commanded_speed(1.0, 0.0, 0.0), 0.0, kTol);
  // Past the end (s > l, d_end < 0): stop, never a reverse speed.
  EXPECT_EQ(sc.commanded_speed(1.0, 0.0, -0.3), 0.0);
}

TEST(SpeedControl, FeedforwardMatchesDragModel)
{
  SpeedControl sc(kSpeed);
  // 1.0 m/s needs 4 + 6 = 10 N in total, 5 N per thruster: throttle 0.1.
  EXPECT_NEAR(sc.feedforward_throttle(1.0), 0.1, kTol);
  // On speed, no integral: the throttle is the feedforward alone.
  EXPECT_NEAR(sc.update(1.0, 1.0, 0.0, kDt), 0.1, kTol);
}

TEST(SpeedControl, ThrottleNotNegativeWhileTurning)
{
  SpeedControl sc(kSpeed);
  // 10.2: psi~ = 1.0 rad at 0.6 m/s. u_cmd = 0.324 m/s, so the PI term would
  // brake: raw throttle = 0.0193 + 0.3 (0.324 - 0.6) = -0.0635.
  const double u_cmd = sc.commanded_speed(0.6, 1.0, 50.0);
  EXPECT_EQ(sc.update(u_cmd, 0.6, 1.0, kDt), 0.0);
  // The braking error is not integrated while held at the limit.
  EXPECT_EQ(sc.integral(), 0.0);
  // Just above psi_turn the same holds; the sign of psi~ does not matter.
  EXPECT_GE(sc.update(0.0, 0.6, -0.31, kDt), 0.0);
}

TEST(SpeedControl, NegativeThrottleAllowedWhenStraight)
{
  SpeedControl sc(kSpeed);
  // psi~ = 0.1 rad (< psi_turn), 0.5 m/s too fast: brake.
  const double u_cmd = sc.commanded_speed(1.0, 0.1, 50.0);
  const double raw = sc.feedforward_throttle(u_cmd) + 0.3 * (u_cmd - 1.5);
  EXPECT_NEAR(sc.update(u_cmd, 1.5, 0.1, 0.0), raw, kTol);  // dt = 0: no I
  EXPECT_LT(raw, 0.0);
  // Far too fast: clamped at min_throttle.
  EXPECT_NEAR(sc.update(u_cmd, 3.0, 0.1, 0.0), -0.3, kTol);
}

TEST(SpeedControl, IntegratesError)
{
  SpeedControl sc(kSpeed);
  // 0.2 m/s too slow for 1 s in 20 steps: I = 0.2 m.
  for (int k = 0; k < 20; ++k) {
    sc.update(0.5, 0.3, 0.0, kDt);
  }
  EXPECT_NEAR(sc.integral(), 0.2, 1e-12);
  const double expected = sc.feedforward_throttle(0.5) + 0.3 * 0.2 + 0.05 * 0.2;
  EXPECT_NEAR(sc.update(0.5, 0.3, 0.0, 0.0), expected, kTol);
  // dt <= 0 (repeated or out-of-order stamp) does not integrate.
  sc.update(0.5, 0.3, 0.0, -kDt);
  EXPECT_NEAR(sc.integral(), 0.2, 1e-12);
  sc.reset();
  EXPECT_EQ(sc.integral(), 0.0);
}

TEST(SpeedControl, AntiWindup)
{
  SpeedControl sc(kSpeed);
  // u_cmd = 3 m/s from standstill: raw = 0.66 + 0.9 > 1, held at max_throttle
  // for 100 s. Without anti-windup I would reach 300 m.
  for (int k = 0; k < 2000; ++k) {
    EXPECT_EQ(sc.update(3.0, 0.0, 0.0, kDt), 1.0);
  }
  EXPECT_EQ(sc.integral(), 0.0);
  // The error reverses (boat too fast): the throttle leaves the limit at once
  // and the error, which pulls away from the limit, is integrated again.
  EXPECT_LT(sc.update(3.0, 3.5, 0.0, kDt), 1.0);
  EXPECT_NEAR(sc.integral(), -0.5 * kDt, 1e-12);
}

TEST(SpeedControl, IntegralFrozenAtEitherLimit)
{
  SpeedControl sc(kSpeed);
  // Build up some integral, then drive the output into each limit.
  for (int k = 0; k < 20; ++k) {
    sc.update(0.5, 0.3, 0.0, kDt);
  }
  const double before = sc.integral();
  sc.update(0.0, 3.0, 0.0, kDt);  // held at min_throttle, error -3
  EXPECT_NEAR(sc.integral(), before, 1e-12);  // pushes further down: frozen
  sc.update(3.0, 0.0, 0.0, kDt);  // held at max_throttle, error +3
  EXPECT_NEAR(sc.integral(), before, 1e-12);  // pushes further up: frozen
}

TEST(SpeedControl, InvalidGainsThrow)
{
  SpeedGains g = kSpeed;
  g.max_thrust = 0.0;
  EXPECT_THROW(SpeedControl{g}, std::invalid_argument);
  g = kSpeed;
  g.speed_ki = -0.1;
  EXPECT_THROW(SpeedControl{g}, std::invalid_argument);
  g = kSpeed;
  g.min_throttle = 0.1;
  EXPECT_THROW(SpeedControl{g}, std::invalid_argument);
  g = kSpeed;
  g.max_throttle = 0.0;
  EXPECT_THROW(SpeedControl{g}, std::invalid_argument);
  g = kSpeed;
  g.drag_quadratic = NAN;
  EXPECT_THROW(SpeedControl{g}, std::invalid_argument);
}

}  // namespace
