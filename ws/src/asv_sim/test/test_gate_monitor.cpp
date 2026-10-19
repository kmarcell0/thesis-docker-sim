// Unit tests for GateMonitor (M1.2d), on the Task 1 gates of
// milestone1_plan.md section 2 [REPO] and the boat size of config/asv.yaml
// [REPO]; margin 0.5 m and capture 2 m [EST], section 6.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "asv_sim/gate_monitor.hpp"

namespace
{

using asv_sim::BoatFootprint;
using asv_sim::Buoy;
using asv_sim::Gate;
using asv_sim::GateDirection;
using asv_sim::GateEvent;
using asv_sim::GateMonitor;
using asv_sim::GateResult;
using asv_sim::Point2;

constexpr double kTol = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kSurmark = 0.2286;  // m, models/rb26_surmark_*/model.sdf
constexpr double kA0 = 0.1015;       // m, models/rb26_a0_*/model.sdf
constexpr double kA2 = 0.184;        // m, models/rb26_a2_*/model.sdf

const BoatFootprint kBoat{1.60, 0.44};  // hull length; 0.66/2 + 0.11

Gate gate_at(const char * name, double x)
{
  return Gate{name, Buoy{std::string(name) + "_red", {x, 1.524}, kSurmark},
    Buoy{std::string(name) + "_green", {x, -1.524}, kSurmark}};
}

GateMonitor task1_monitor(const std::vector<Buoy> & others = {})
{
  return GateMonitor({gate_at("gate1", 5.0), gate_at("gate2", 13.0)}, others, kBoat, 0.5, 2.0);
}

// Drives in a straight line from (x0, y) to (x1, y) in steps of 0.05 m and
// collects every event.
std::vector<GateEvent> drive(GateMonitor & m, double x0, double x1, double y)
{
  const double heading = x1 > x0 ? 0.0 : kPi;
  const int n = static_cast<int>(std::round(std::fabs(x1 - x0) / 0.05));
  std::vector<GateEvent> all;
  for (int k = 0; k <= n; ++k) {
    const double x = x0 + (x1 - x0) * k / n;
    for (const auto & e : m.update({x, y}, heading)) {
      all.push_back(e);
    }
  }
  return all;
}

// ---- crossings ------------------------------------------------------------

TEST(GateMonitor, OutboundThroughBothGatesPasses)
{
  auto m = task1_monitor();
  const auto events = drive(m, 0.0, 17.0, 0.0);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].gate, "gate1");
  EXPECT_EQ(events[1].gate, "gate2");
  for (const auto & e : events) {
    EXPECT_EQ(e.direction, GateDirection::OUTBOUND);
    EXPECT_NEAR(e.offset, 0.0, kTol);
    EXPECT_EQ(e.result, GateResult::PASS);
  }
}

TEST(GateMonitor, InboundDirection)
{
  auto m = task1_monitor();
  const auto events = drive(m, 17.0, -1.0, 0.0);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].gate, "gate2");
  EXPECT_EQ(events[1].gate, "gate1");
  EXPECT_EQ(events[0].direction, GateDirection::INBOUND);
  EXPECT_EQ(events[1].result, GateResult::PASS);
}

TEST(GateMonitor, FirstUpdateGivesNoEvent)
{
  auto m = task1_monitor();
  EXPECT_TRUE(m.update({5.0, 0.0}, 0.0).empty());
}

TEST(GateMonitor, OffsetSignAndInterpolation)
{
  // A diagonal step: the crossing point is interpolated, not the end point.
  auto m = task1_monitor();
  m.update({4.0, 0.0}, 0.0);
  const auto events = m.update({6.0, 0.4}, 0.0);  // crosses x = 5 at y = 0.2
  ASSERT_EQ(events.size(), 1u);
  EXPECT_NEAR(events[0].offset, 0.2, kTol);  // > 0: towards red (north)
}

TEST(GateMonitor, BandEdgeSection56)
{
  // Band: 1.524 - 0.2286 - 0.44 - 0.5 = 0.3554 m.
  struct Case {double y; GateResult expected;};
  for (const Case c : {Case{0.355, GateResult::PASS}, Case{-0.355, GateResult::PASS},
      Case{0.356, GateResult::FAIL_CLOSE}, Case{-0.356, GateResult::FAIL_CLOSE},
      Case{1.5, GateResult::FAIL_CLOSE}})
  {
    auto m = task1_monitor();
    const auto events = drive(m, 4.0, 6.0, c.y);
    ASSERT_EQ(events.size(), 1u) << "y = " << c.y;
    EXPECT_NEAR(events[0].offset, c.y, kTol);
    EXPECT_EQ(events[0].result, c.expected) << "y = " << c.y;
  }
}

TEST(GateMonitor, OutsideTheBuoys)
{
  {
    auto m = task1_monitor();
    const auto events = drive(m, 4.0, 6.0, 2.0);  // north of the red buoy
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].result, GateResult::FAIL_OUTSIDE);
  }
  {
    auto m = task1_monitor();
    const auto events = drive(m, 4.0, 6.0, -3.5);  // 1.976 m beyond green: captured
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].result, GateResult::FAIL_OUTSIDE);
  }
  {
    auto m = task1_monitor();
    EXPECT_TRUE(drive(m, 4.0, 6.0, 4.0).empty());  // beyond 1.524 + 2 m
  }
}

TEST(GateMonitor, LandingExactlyOnTheLineCountsOnce)
{
  auto m = task1_monitor();
  m.update({4.9, 0.0}, 0.0);
  const auto on_line = m.update({5.0, 0.0}, 0.0);
  const auto after = m.update({5.1, 0.0}, 0.0);
  EXPECT_EQ(on_line.size() + after.size(), 1u);
}

TEST(GateMonitor, EveryCrossingIsReported)
{
  auto m = task1_monitor();
  m.update({4.9, 0.0}, 0.0);
  int n = 0;
  n += static_cast<int>(m.update({5.1, 0.0}, 0.0).size());  // out
  const auto back = m.update({4.95, 0.0}, kPi);              // in
  n += static_cast<int>(back.size());
  n += static_cast<int>(m.update({5.2, 0.0}, 0.0).size());  // out
  EXPECT_EQ(n, 3);
  ASSERT_EQ(back.size(), 1u);
  EXPECT_EQ(back[0].direction, GateDirection::INBOUND);
}

TEST(GateMonitor, UnevenBandWithMixedRadii)
{
  // Red A2 (0.184 m), green Sur-Mark (0.2286 m), 3.048 m apart:
  // red side 1.524 - 0.184 - 0.94 = 0.400 m, green side 0.3554 m.
  const Gate g{"mixed", Buoy{"r", {5.0, 1.524}, kA2}, Buoy{"g", {5.0, -1.524}, kSurmark}};
  for (const double y : {0.399, -0.355}) {
    GateMonitor m({g}, {}, kBoat, 0.5, 2.0);
    const auto e = drive(m, 4.0, 6.0, y);
    ASSERT_EQ(e.size(), 1u);
    EXPECT_EQ(e[0].result, GateResult::PASS) << "y = " << y;
  }
  for (const double y : {0.401, -0.356}) {
    GateMonitor m({g}, {}, kBoat, 0.5, 2.0);
    const auto e = drive(m, 4.0, 6.0, y);
    ASSERT_EQ(e.size(), 1u);
    EXPECT_EQ(e[0].result, GateResult::FAIL_CLOSE) << "y = " << y;
  }
}

// ---- clearance ------------------------------------------------------------

TEST(GateMonitor, ClearanceSideAndBow)
{
  const auto m = task1_monitor();
  const Buoy red{"red", {5.0, 1.524}, kSurmark};
  // Heading east: the buoy is beside the boat, 1.524 - 0.44 - 0.2286.
  EXPECT_NEAR(m.clearance(red, {5.0, 0.0}, 0.0), 0.8554, kTol);
  // Heading north: the bow points at it, 1.524 - 0.80 - 0.2286.
  EXPECT_NEAR(m.clearance(red, {5.0, 0.0}, kPi / 2), 0.4954, kTol);
  // Diagonal to the bow-left corner (0.8, 0.44): from buoy at (1.8, 1.44).
  const Buoy corner{"c", {6.8, 1.44}, kSurmark};
  EXPECT_NEAR(m.clearance(corner, {5.0, 0.0}, 0.0), std::hypot(1.0, 1.0) - kSurmark, kTol);
}

TEST(GateMonitor, ClearanceInsideRectangle)
{
  const auto m = task1_monitor();
  const Buoy b{"b", {5.3, 0.1}, kSurmark};
  EXPECT_NEAR(m.clearance(b, {5.0, 0.0}, 0.0), -kSurmark, kTol);
}

TEST(GateMonitor, CollisionDetected)
{
  auto m = task1_monitor();
  m.update({5.0, 1.0}, 0.0);  // 0.084 - 0.2286 = -0.1446 to gate1_red
  EXPECT_TRUE(m.collision());
  const auto c = m.closest();
  EXPECT_EQ(c.name, "gate1_red");
  EXPECT_NEAR(c.min_clearance, -0.1446, kTol);
}

TEST(GateMonitor, MinimumIsKeptAndNoCollisionOnCentreLine)
{
  auto m = task1_monitor();
  drive(m, 0.0, 17.0, 0.0);
  EXPECT_FALSE(m.collision());
  ASSERT_EQ(m.clearances().size(), 4u);
  for (const auto & c : m.clearances()) {
    EXPECT_NEAR(c.min_clearance, 0.8554, kTol) << c.name;
  }
}

TEST(GateMonitor, OtherBuoysAreTracked)
{
  // Task 2 channel gate 1 and Task 3 gate in world coordinates (M1.2d brief).
  const std::vector<Buoy> others{
    {"t2_ch1_red", {20.781, 6.0}, kA0}, {"t2_ch1_green", {23.219, 6.0}, kA0},
    {"t3_red", {30.0, -4.476}, kA2}, {"t3_green", {30.0, -7.524}, kA2}};
  auto m = task1_monitor(others);
  ASSERT_EQ(m.clearances().size(), 8u);
  m.update({25.5, -2.0}, -kPi / 2);  // W4, heading south
  // t3_red - p = (4.5, -2.476); rotated by -psi = +pi/2 into the boat frame:
  // (2.476, 4.5), ahead and to the left; nearest rectangle point is the
  // bow-left corner (0.8, 0.44): c = hypot(1.676, 4.06) - 0.184.
  const auto & c = m.clearances();
  EXPECT_NEAR(c[6].min_clearance, std::hypot(1.676, 4.06) - kA2, 1e-9);
  // The gate buoys stay the first four, in gate order.
  EXPECT_EQ(c[0].name, "gate1_red");
  EXPECT_EQ(c[4].name, "t2_ch1_red");
  EXPECT_EQ(m.closest().name, "t3_red");
}

// ---- invalid input --------------------------------------------------------

TEST(GateMonitor, InvalidArgumentsThrow)
{
  const Gate ok = gate_at("g", 5.0);
  const Gate same{"s", Buoy{"r", {5.0, 0.0}, kSurmark}, Buoy{"g", {5.0, 0.0}, kSurmark}};
  Gate zero_radius = ok;
  zero_radius.red.radius = 0.0;
  EXPECT_THROW(GateMonitor({same}, {}, kBoat, 0.5, 2.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({zero_radius}, {}, kBoat, 0.5, 2.0), std::invalid_argument);
  EXPECT_THROW(
    GateMonitor({ok}, {Buoy{"x", {0.0, 0.0}, -0.1}}, kBoat, 0.5, 2.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({ok}, {}, BoatFootprint{0.0, 0.44}, 0.5, 2.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({ok}, {}, BoatFootprint{1.6, 0.0}, 0.5, 2.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({ok}, {}, kBoat, -0.1, 2.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({ok}, {}, kBoat, 0.5, -1.0), std::invalid_argument);
  EXPECT_THROW(GateMonitor({ok}, {}, kBoat, std::nan(""), 2.0), std::invalid_argument);
}

}  // namespace
