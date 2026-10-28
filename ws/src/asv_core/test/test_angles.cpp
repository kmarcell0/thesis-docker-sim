// Unit tests for the angle helpers (M1.2b).
#include <gtest/gtest.h>

#include <cmath>

#include "asv_core/angles.hpp"

namespace
{

using asv_core::wrap_angle;
using asv_core::yaw_from_quaternion;

constexpr double kPi = M_PI;
constexpr double kTol = 1e-12;

// Quaternion of the yaw-pitch-roll (Z-Y-X) rotation used by ROS / tf2.
struct Quat
{
  double w, x, y, z;
};

Quat from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

double yaw_of(const Quat & q) {return yaw_from_quaternion(q.w, q.x, q.y, q.z);}

TEST(WrapAngle, InsideRangeUnchanged)
{
  EXPECT_NEAR(wrap_angle(0.0), 0.0, kTol);
  EXPECT_NEAR(wrap_angle(1.0), 1.0, kTol);
  EXPECT_NEAR(wrap_angle(-1.0), -1.0, kTol);
}

TEST(WrapAngle, RangeIsHalfOpenAtMinusPi)
{
  // (-pi, pi]: both ends of the circle map to +pi.
  EXPECT_DOUBLE_EQ(wrap_angle(kPi), kPi);
  EXPECT_DOUBLE_EQ(wrap_angle(-kPi), kPi);
  EXPECT_DOUBLE_EQ(wrap_angle(3 * kPi), kPi);
}

TEST(WrapAngle, OutsideRangeWrapped)
{
  EXPECT_NEAR(wrap_angle(1.5 * kPi), -0.5 * kPi, kTol);
  EXPECT_NEAR(wrap_angle(-1.5 * kPi), 0.5 * kPi, kTol);
  EXPECT_NEAR(wrap_angle(2 * kPi + 0.3), 0.3, kTol);
}

TEST(WrapAngle, DifferenceAcrossPlusMinusPi)
{
  // From heading 3.0 to heading -3.0 is a short left turn of 2 pi - 6 rad,
  // not a right turn of 6 rad.
  EXPECT_NEAR(wrap_angle(-3.0 - 3.0), 2 * kPi - 6.0, kTol);
  EXPECT_NEAR(wrap_angle(3.0 - (-3.0)), 6.0 - 2 * kPi, kTol);
}

TEST(YawFromQuaternion, PureYaw)
{
  EXPECT_NEAR(yaw_of({1.0, 0.0, 0.0, 0.0}), 0.0, kTol);
  for (double yaw : {0.7, -0.7, kPi - 0.01, -kPi + 0.01}) {
    EXPECT_NEAR(yaw_of(from_rpy(0.0, 0.0, yaw)), yaw, kTol) << "yaw " << yaw;
  }
}

TEST(YawFromQuaternion, RollAndPitchDoNotChangeYaw)
{
  EXPECT_NEAR(yaw_of(from_rpy(0.1, 0.05, 1.0)), 1.0, kTol);
  EXPECT_NEAR(yaw_of(from_rpy(-0.1, 0.05, -2.5)), -2.5, kTol);
}

}  // namespace
