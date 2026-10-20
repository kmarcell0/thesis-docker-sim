// Unit tests for asv_sim::apply_key() (M1.8).
#include <gtest/gtest.h>

#include <string>

#include "asv_sim/teleop.hpp"

namespace
{

using asv_sim::apply_key;
using asv_sim::TeleopLevels;
using asv_sim::TeleopSteps;
using asv_sim::ThrusterLimits;

// The current values in config/asv.yaml (thrusters section) [REPO].
const ThrusterLimits kLimits{50.0, -40.0, 1.5708};
// The values in config/teleop.yaml [REPO].
const TeleopSteps kSteps{0.1, 0.1, 0.1};

constexpr double kTol = 1e-9;

// Feeds the keys one after the other, starting from zero levels.
TeleopLevels press(const std::string & keys)
{
  TeleopLevels l;
  for (const char k : keys) {
    l = apply_key(l, k, kSteps, kLimits).levels;
  }
  return l;
}

TEST(Teleop, EachKeyMovesOneLevelBySign)
{
  EXPECT_NEAR(press("w").throttle, 0.1, kTol);
  EXPECT_NEAR(press("s").throttle, -0.1, kTol);
  EXPECT_NEAR(press("a").steer, 0.1, kTol);    // A = left = steer > 0
  EXPECT_NEAR(press("d").steer, -0.1, kTol);
  EXPECT_NEAR(press("q").spin, 0.1, kTol);     // Q = left = spin > 0
  EXPECT_NEAR(press("e").spin, -0.1, kTol);
  const auto l = press("w");
  EXPECT_NEAR(l.steer, 0.0, kTol);
  EXPECT_NEAR(l.spin, 0.0, kTol);
}

TEST(Teleop, LevelsAddUpAndStaySet)
{
  const auto l = press("wwwaqq");
  EXPECT_NEAR(l.throttle, 0.3, kTol);
  EXPECT_NEAR(l.steer, 0.1, kTol);
  EXPECT_NEAR(l.spin, 0.2, kTol);
  EXPECT_NEAR(press("wwws").throttle, 0.2, kTol);
}

TEST(Teleop, UpperCaseSameAsLowerCase)
{
  const auto lower = press("wwadqe");
  const auto upper = press("WWADQE");
  EXPECT_NEAR(upper.throttle, lower.throttle, kTol);
  EXPECT_NEAR(upper.steer, lower.steer, kTol);
  EXPECT_NEAR(upper.spin, lower.spin, kTol);
}

TEST(Teleop, SpaceSetsAllToZero)
{
  const auto l = press("wwwaaqq ");
  EXPECT_EQ(l.throttle, 0.0);
  EXPECT_EQ(l.steer, 0.0);
  EXPECT_EQ(l.spin, 0.0);
}

TEST(Teleop, OtherKeysChangeNothing)
{
  const TeleopLevels start{0.3, -0.2, 0.1};
  for (const char k : std::string("zr1\n\t\x1b")) {
    const auto u = apply_key(start, k, kSteps, kLimits);
    EXPECT_FALSE(u.quit) << int(k);
    EXPECT_EQ(u.levels.throttle, start.throttle) << int(k);
    EXPECT_EQ(u.levels.steer, start.steer) << int(k);
    EXPECT_EQ(u.levels.spin, start.spin) << int(k);
  }
}

TEST(Teleop, QuitKeysZeroAndQuit)
{
  const TeleopLevels start{0.5, 0.3, -0.2};
  for (const char k : {'x', 'X', '\x03'}) {
    const auto u = apply_key(start, k, kSteps, kLimits);
    EXPECT_TRUE(u.quit) << int(k);
    EXPECT_EQ(u.levels.throttle, 0.0);
    EXPECT_EQ(u.levels.steer, 0.0);
    EXPECT_EQ(u.levels.spin, 0.0);
  }
  EXPECT_FALSE(apply_key(start, 'w', kSteps, kLimits).quit);
}

TEST(Teleop, ForwardLimits)
{
  const auto l = press(std::string(15, 'w') + std::string(20, 'a') + std::string(15, 'q'));
  EXPECT_NEAR(l.throttle, 1.0, kTol);
  EXPECT_NEAR(l.steer, kLimits.steer_limit, kTol);
  EXPECT_NEAR(l.spin, 1.0, kTol);
}

TEST(Teleop, ReverseLimits)
{
  // Reverse throttle ends at min_thrust / max_thrust = -40 / 50 = -0.8.
  const auto l = press(std::string(15, 's') + std::string(20, 'd') + std::string(15, 'e'));
  EXPECT_NEAR(l.throttle, -0.8, kTol);
  EXPECT_NEAR(l.steer, -kLimits.steer_limit, kTol);
  EXPECT_NEAR(l.spin, -1.0, kTol);
}

TEST(Teleop, BackFromTheLimitTakesOneStep)
{
  // The clamp must not store levels beyond the limit: one S after full
  // throttle gives 0.9, not 1.4.
  EXPECT_NEAR(press(std::string(15, 'w') + "s").throttle, 0.9, kTol);
}

}  // namespace
