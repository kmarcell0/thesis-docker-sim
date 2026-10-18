// Unit tests for Route and LosGuidance (M1.2b), on the Task 1 route of
// milestone1_plan.md section 2 [REPO].
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "asv_sim/guidance.hpp"

namespace
{

using asv_sim::LosGuidance;
using asv_sim::Point2;
using asv_sim::Route;

constexpr double kTol = 1e-9;
constexpr double kSwitch = 1.5;     // m, R_switch (task1_route.yaml)
constexpr double kLookahead = 3.0;  // m, Delta (task1_route.yaml)

// W0 (spawn) ... W9.
const std::vector<Point2> kTask1{
  {0.0, 0.0}, {17.0, 0.0}, {22.0, 2.5}, {25.0, 1.0}, {25.5, -2.0},
  {23.0, -4.0}, {20.0, -2.5}, {17.0, -0.5}, {15.0, 0.0}, {-1.0, 0.0}};

// Feed the waypoints W1..W_{last} one by one: each one switches one segment.
void advance_to_segment(Route & route, std::size_t segment)
{
  for (std::size_t k = 1; k <= segment; ++k) {
    route.update(kTask1[k]);
  }
  ASSERT_EQ(route.segment(), segment);
}

TEST(Route, AlongAndCrossTrack)
{
  Route route(kTask1, kSwitch);
  route.update({5.0, 0.5});
  EXPECT_EQ(route.segment(), 0u);
  EXPECT_NEAR(route.along_track(), 5.0, kTol);
  EXPECT_NEAR(route.cross_track(), 0.5, kTol);  // left of the path: e > 0
  route.update({5.0, -0.5});
  EXPECT_NEAR(route.cross_track(), -0.5, kTol);
}

TEST(Route, SegmentGeometry)
{
  Route route(kTask1, kSwitch);
  EXPECT_EQ(route.num_segments(), 9u);
  EXPECT_NEAR(route.segment_direction(0), 0.0, kTol);
  EXPECT_NEAR(route.segment_direction(1), std::atan2(2.5, 5.0), kTol);
  EXPECT_NEAR(route.segment_direction(1), 0.4636476, 1e-7);
  EXPECT_NEAR(route.segment_direction(8), M_PI, kTol);
  EXPECT_NEAR(route.segment_length(1), std::hypot(5.0, 2.5), kTol);
}

TEST(Route, SwitchesWhenPassedAlongThePath)
{
  Route route(kTask1, kSwitch);
  route.update({15.4, 0.0});  // s = 15.4 < 17 - 1.5
  EXPECT_EQ(route.segment(), 0u);
  route.update({15.5, 0.0});  // s = 15.5 = 17 - 1.5
  EXPECT_EQ(route.segment(), 1u);
}

TEST(Route, MissedWaypointStillSwitches)
{
  // 3.16 m from W1, outside a 1.5 m circle, but past it along the path.
  Route route(kTask1, kSwitch);
  route.update({16.0, 3.0});
  EXPECT_EQ(route.segment(), 1u);
}

TEST(Route, AtMostOneSwitchPerUpdate)
{
  // A jump to W3 would pass several waypoints; only one switch per update.
  Route route(kTask1, kSwitch);
  route.update(kTask1[3]);
  EXPECT_EQ(route.segment(), 1u);
  route.update(kTask1[3]);
  EXPECT_EQ(route.segment(), 2u);
}

TEST(Route, RemainingDistanceAlongTheRoute)
{
  double total = 0.0;
  for (std::size_t k = 0; k + 1 < kTask1.size(); ++k) {
    total += std::hypot(kTask1[k + 1].x - kTask1[k].x, kTask1[k + 1].y - kTask1[k].y);
  }
  Route route(kTask1, kSwitch);
  route.update(kTask1[0]);
  EXPECT_NEAR(route.remaining_distance(), total, kTol);
  EXPECT_NEAR(route.remaining_distance(), 57.21, 0.005);
  route.update({5.0, 0.5});
  EXPECT_NEAR(route.remaining_distance(), total - 5.0, kTol);
}

TEST(Route, FinishedOnLastSegmentBySwitchRule)
{
  Route route(kTask1, kSwitch);
  advance_to_segment(route, 8);
  route.update({0.6, 0.0});  // s = 14.4
  EXPECT_FALSE(route.finished());
  EXPECT_EQ(route.segment(), 8u);
  route.update({0.5, 0.0});  // s = 14.5 = 16 - 1.5
  EXPECT_TRUE(route.finished());
  EXPECT_EQ(route.segment(), 8u);
  EXPECT_NEAR(route.remaining_distance(), 1.5, kTol);
  route.update({5.0, 0.0});  // stays finished
  EXPECT_TRUE(route.finished());
}

TEST(Route, BadInputThrows)
{
  EXPECT_THROW(Route({{0.0, 0.0}}, kSwitch), std::invalid_argument);
  EXPECT_THROW(Route({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}}, kSwitch), std::invalid_argument);
  EXPECT_THROW(Route(kTask1, -0.1), std::invalid_argument);
  EXPECT_THROW(LosGuidance(0.0), std::invalid_argument);
}

TEST(LosGuidance, OnLeftAndRightOfThePath)
{
  Route route(kTask1, kSwitch);
  const LosGuidance los(kLookahead);

  route.update({5.0, 0.0});
  auto out = los.compute(route, {5.0, 0.0});
  EXPECT_NEAR(out.desired_heading, 0.0, kTol);
  EXPECT_NEAR(out.target.x, 8.0, kTol);
  EXPECT_NEAR(out.target.y, 0.0, kTol);

  route.update({5.0, 1.0});  // left of the path: turn right (psi_d < 0)
  out = los.compute(route, {5.0, 1.0});
  EXPECT_NEAR(out.desired_heading, std::atan2(-1.0, 3.0), kTol);
  EXPECT_NEAR(out.desired_heading, -0.3217506, 1e-7);

  route.update({5.0, -1.0});
  out = los.compute(route, {5.0, -1.0});
  EXPECT_NEAR(out.desired_heading, 0.3217506, 1e-7);
}

TEST(LosGuidance, EqualsFossenFormOnAStraightSegment)
{
  // psi_d = gamma_i + atan2(-e, Delta) while the target stays on segment i.
  Route route(kTask1, kSwitch);
  advance_to_segment(route, 1);
  const Point2 p{18.0, 1.0};  // s = 1.34, s + Delta = 4.34 < l_1 = 5.59
  route.update(p);
  ASSERT_EQ(route.segment(), 1u);
  const auto out = LosGuidance(kLookahead).compute(route, p);
  EXPECT_NEAR(
    out.desired_heading,
    route.segment_direction(1) + std::atan2(-route.cross_track(), kLookahead), kTol);
}

TEST(LosGuidance, LookaheadCarriedOntoNextSegment)
{
  // s = 15 on segment 0: 2 m to W1, the last 1 m goes along segment 1.
  Route route(kTask1, kSwitch);
  const Point2 p{15.0, 0.0};
  route.update(p);
  ASSERT_EQ(route.segment(), 0u);
  const auto out = LosGuidance(kLookahead).compute(route, p);
  const double l1 = std::hypot(5.0, 2.5);
  EXPECT_NEAR(out.target.x, 17.0 + 5.0 / l1, kTol);
  EXPECT_NEAR(out.target.y, 2.5 / l1, kTol);
  EXPECT_NEAR(out.target.x, 17.894, 0.0005);
  EXPECT_NEAR(out.target.y, 0.447, 0.0005);
  // The target lies on segment 1 (zero cross-track distance to it).
  const double ex = out.target.x - 17.0, ey = out.target.y - 0.0;
  EXPECT_NEAR((-2.5 * ex + 5.0 * ey) / l1, 0.0, kTol);
}

TEST(LosGuidance, TargetStopsAtLastWaypoint)
{
  Route route(kTask1, kSwitch);
  advance_to_segment(route, 8);
  const Point2 p{1.0, 0.0};  // s = 14, s + Delta = 17 > l_8 = 16
  route.update(p);
  const auto out = LosGuidance(kLookahead).compute(route, p);
  EXPECT_NEAR(out.target.x, -1.0, kTol);
  EXPECT_NEAR(out.target.y, 0.0, kTol);
}

TEST(LosGuidance, TargetNotBehindTheStart)
{
  Route route(kTask1, kSwitch);
  const Point2 p{-5.0, 0.0};  // s = -5, s + Delta = -2: target clamped to W0
  route.update(p);
  const auto out = LosGuidance(kLookahead).compute(route, p);
  EXPECT_NEAR(out.target.x, 0.0, kTol);
  EXPECT_NEAR(out.target.y, 0.0, kTol);
}

TEST(LosGuidance, WrappedAcrossPlusMinusPi)
{
  // Segment 8 heads west (gamma = pi); left of the path is south.
  Route route(kTask1, kSwitch);
  advance_to_segment(route, 8);
  const LosGuidance los(kLookahead);

  route.update({10.0, -0.5});
  EXPECT_NEAR(route.cross_track(), 0.5, kTol);
  auto out = los.compute(route, {10.0, -0.5});
  EXPECT_NEAR(out.desired_heading, std::atan2(0.5, -3.0), kTol);
  EXPECT_NEAR(out.desired_heading, 2.976, 0.0005);

  route.update({10.0, 0.5});
  out = los.compute(route, {10.0, 0.5});
  EXPECT_NEAR(out.desired_heading, -2.976, 0.0005);
}

}  // namespace
