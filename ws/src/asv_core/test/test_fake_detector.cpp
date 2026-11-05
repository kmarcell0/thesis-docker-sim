// Tests of the fake detector (M3.1): visibility, geometry, noise statistics,
// misses, false alarms, colour mix-ups, seeds, and the scan grid.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>

#include "asv_core/fake_detector.hpp"

namespace
{

using asv_core::BuoyColour;
using asv_core::Detection;
using asv_core::DetectorCamera;
using asv_core::DetectorObject;
using asv_core::DetectorParams;
using asv_core::FakeDetector;
using asv_core::Pose2;
using asv_core::ScanGrid;

constexpr double kPi = 3.14159265358979323846;

// Geometry like the boat (asv_frames.yaml): lidar 0.25 m forward, one camera
// looking ahead with 72 deg. No noise unless a test sets it.
DetectorParams test_params()
{
  DetectorParams p;
  p.lidar_x = 0.25;
  p.lidar_y = 0.0;
  p.cameras.push_back(DetectorCamera{0.42, 0.0, 0.0, 72.0 * kPi / 180.0});
  return p;
}

DetectorObject object(std::uint32_t id, double x, double y, BuoyColour colour = BuoyColour::red)
{
  DetectorObject o;
  o.id = id;
  o.x = x;
  o.y = y;
  o.radius = 0.2286;
  o.colour = colour;
  o.lidar_range = 18.75;
  o.camera_range = {20.1};
  return o;
}

const Detection * find_id(const std::vector<Detection> & ds, std::uint32_t id)
{
  for (const auto & d : ds) {
    if (d.true_id && *d.true_id == id) {
      return &d;
    }
  }
  return nullptr;
}

// Mean and sample standard deviation.
void moments(const std::vector<double> & v, double & mean, double & sd)
{
  double s = 0.0;
  for (double x : v) {
    s += x;
  }
  mean = s / static_cast<double>(v.size());
  double ss = 0.0;
  for (double x : v) {
    ss += (x - mean) * (x - mean);
  }
  sd = std::sqrt(ss / static_cast<double>(v.size() - 1));
}

TEST(FakeDetector, LidarRangeIsMeasuredFromTheLidar)
{
  // Lidar at x = 0.25 in base_link; limit 18.75 m. Boat at the origin facing
  // east: an object at x = 0.25 + 18.74 is seen, at 0.25 + 18.76 not.
  FakeDetector det(test_params(), {object(0, 0.25 + 18.74, 0.0), object(1, 0.25 + 18.76, 0.0)}, 1);
  const auto ds = det.scan(Pose2{});
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(*ds[0].true_id, 0u);
  // The range itself is from the base_link origin.
  EXPECT_NEAR(ds[0].range, 18.99, 1e-12);
}

TEST(FakeDetector, LidarSeesAllAround)
{
  // 360 deg lidar: objects behind and to the sides are reported too.
  FakeDetector det(
    test_params(),
    {object(0, -5.0, 0.0), object(1, 0.0, 5.0), object(2, 0.0, -5.0), object(3, 5.0, 0.0)}, 1);
  EXPECT_EQ(det.scan(Pose2{}).size(), 4u);
}

TEST(FakeDetector, BearingIsPositiveToTheLeftAndFollowsYaw)
{
  // Boat at (10, 20) facing north (yaw pi/2). A buoy 5 m west is on its left
  // (+pi/2); one 5 m north is straight ahead (0); one 5 m east on its right.
  FakeDetector det(
    test_params(),
    {object(0, 5.0, 20.0), object(1, 10.0, 25.0), object(2, 15.0, 20.0)}, 1);
  const auto ds = det.scan(Pose2{10.0, 20.0, kPi / 2});
  ASSERT_EQ(ds.size(), 3u);
  EXPECT_NEAR(find_id(ds, 0)->bearing, kPi / 2, 1e-12);
  EXPECT_NEAR(find_id(ds, 1)->bearing, 0.0, 1e-12);
  EXPECT_NEAR(find_id(ds, 2)->bearing, -kPi / 2, 1e-12);
  for (const auto & d : ds) {
    EXPECT_NEAR(d.range, 5.0, 1e-12);
  }
  // Sorted by bearing, not by id.
  EXPECT_EQ(*ds[0].true_id, 2u);
  EXPECT_EQ(*ds[2].true_id, 0u);
}

TEST(FakeDetector, BearingBehindStaysWrapped)
{
  // A buoy straight behind has bearing +-pi; with noise every reported
  // bearing must stay in (-pi, pi] and be close to pi in the wrapped sense.
  DetectorParams p = test_params();
  p.bearing_sigma = 0.05;
  FakeDetector det(p, {object(0, -5.0, 0.0)}, 3);
  int left = 0;
  int right = 0;
  for (int k = 0; k < 2000; ++k) {
    const auto ds = det.scan(Pose2{});
    ASSERT_EQ(ds.size(), 1u);
    const double b = ds[0].bearing;
    ASSERT_GT(b, -kPi);
    ASSERT_LE(b, kPi);
    EXPECT_LT(kPi - std::abs(b), 0.3);
    (b > 0 ? left : right)++;
  }
  // Both sides of the cut occur.
  EXPECT_GT(left, 500);
  EXPECT_GT(right, 500);
}

TEST(FakeDetector, ColourOnlyInsideCameraViewAndRange)
{
  // Camera at x = 0.42 facing ahead, hfov 72 deg (half 36 deg), range 20.1 m.
  // id 0: ahead 10 m -> colour; id 1: 35 deg off axis -> colour;
  // id 2: 37 deg off axis -> UNKNOWN; id 3: behind -> UNKNOWN;
  // id 4: ahead, inside lidar range (18.75 from the lidar) but the camera
  // limit is set to 15 m -> UNKNOWN.
  auto at_angle = [](std::uint32_t id, double deg, double dist) {
      const double a = deg * kPi / 180.0;
      return object(id, 0.42 + dist * std::cos(a), dist * std::sin(a));
    };
  auto far = object(4, 17.0, 0.0);
  far.camera_range = {15.0};
  FakeDetector det(
    test_params(),
    {object(0, 10.0, 0.0), at_angle(1, 35.0, 8.0), at_angle(2, 37.0, 8.0),
      object(3, -6.0, 0.0), far}, 1);
  const auto ds = det.scan(Pose2{});
  ASSERT_EQ(ds.size(), 5u);
  EXPECT_EQ(find_id(ds, 0)->colour, BuoyColour::red);
  EXPECT_EQ(find_id(ds, 1)->colour, BuoyColour::red);
  EXPECT_EQ(find_id(ds, 2)->colour, BuoyColour::unknown);
  EXPECT_EQ(find_id(ds, 3)->colour, BuoyColour::unknown);
  EXPECT_EQ(find_id(ds, 4)->colour, BuoyColour::unknown);
}

TEST(FakeDetector, VesselIsFlagged)
{
  auto v = object(7, 6.0, 0.0, BuoyColour::black);
  v.vessel = true;
  FakeDetector det(test_params(), {v}, 1);
  const auto ds = det.scan(Pose2{});
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_TRUE(ds[0].vessel);
  EXPECT_EQ(ds[0].colour, BuoyColour::black);
}

TEST(FakeDetector, NoiseMomentsMatchSettings)
{
  // 1e5 scans of one buoy 10 m ahead: mean error within 3 standard errors of
  // 0 (SE = sigma / sqrt(N)), sample sigma within 3 SE of the setting
  // (SE of a sample sigma = sigma / sqrt(2 (N - 1))).
  DetectorParams p = test_params();
  p.range_sigma = 0.10;
  p.bearing_sigma = 0.02;
  FakeDetector det(p, {object(0, 10.0, 0.0)}, 42);
  const int n = 100000;
  std::vector<double> er;
  std::vector<double> eb;
  er.reserve(n);
  eb.reserve(n);
  for (int k = 0; k < n; ++k) {
    const auto ds = det.scan(Pose2{});
    ASSERT_EQ(ds.size(), 1u);
    er.push_back(ds[0].range - 10.0);
    eb.push_back(ds[0].bearing);
  }
  double mean = 0.0;
  double sd = 0.0;
  moments(er, mean, sd);
  EXPECT_LT(std::abs(mean), 3.0 * 0.10 / std::sqrt(n));
  EXPECT_LT(std::abs(sd - 0.10), 3.0 * 0.10 / std::sqrt(2.0 * (n - 1)));
  moments(eb, mean, sd);
  EXPECT_LT(std::abs(mean), 3.0 * 0.02 / std::sqrt(n));
  EXPECT_LT(std::abs(sd - 0.02), 3.0 * 0.02 / std::sqrt(2.0 * (n - 1)));
}

TEST(FakeDetector, CentroidBiasShortensRange)
{
  DetectorParams p = test_params();
  p.centroid_bias = true;
  FakeDetector det(p, {object(0, 10.0, 0.0)}, 1);
  const auto ds = det.scan(Pose2{});
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_NEAR(ds[0].range, 10.0 - kPi / 4 * 0.2286, 1e-12);
  EXPECT_NEAR(ds[0].bearing, 0.0, 1e-12);
}

TEST(FakeDetector, MissRateMatchesSetting)
{
  // P_miss = 0.2 on 10 buoys over 10^4 scans: miss fraction within 3 SE,
  // SE = sqrt(P (1 - P) / N).
  DetectorParams p = test_params();
  p.miss_probability = 0.2;
  std::vector<DetectorObject> objs;
  for (std::uint32_t i = 0; i < 10; ++i) {
    objs.push_back(object(i, 2.0 + i, 1.0));
  }
  FakeDetector det(p, objs, 5);
  const int scans = 10000;
  long seen = 0;
  for (int k = 0; k < scans; ++k) {
    seen += static_cast<long>(det.scan(Pose2{}).size());
  }
  const double n = 10.0 * scans;
  const double miss = 1.0 - seen / n;
  EXPECT_LT(std::abs(miss - 0.2), 3.0 * std::sqrt(0.2 * 0.8 / n));
}

TEST(FakeDetector, FalseAlarmRateAndPlacement)
{
  // lambda = 0.5 per scan over 10^5 scans, no map objects: mean count within
  // 3 SE (SE = sqrt(lambda / N)); every false alarm has no true id, UNKNOWN
  // colour, range in (0, 18.75], bearing in (-pi, pi]; mean range 18.75 / 2.
  DetectorParams p = test_params();
  p.false_alarm_rate = 0.5;
  p.false_alarm_max_range = 18.75;
  FakeDetector det(p, {}, 9);
  const int scans = 100000;
  long count = 0;
  double range_sum = 0.0;
  double bearing_sum = 0.0;
  for (int k = 0; k < scans; ++k) {
    for (const auto & d : det.scan(Pose2{})) {
      ++count;
      ASSERT_FALSE(d.true_id.has_value());
      ASSERT_EQ(d.colour, BuoyColour::unknown);
      ASSERT_FALSE(d.vessel);
      ASSERT_GT(d.range, 0.0);
      ASSERT_LE(d.range, 18.75);
      ASSERT_GT(d.bearing, -kPi);
      ASSERT_LE(d.bearing, kPi);
      range_sum += d.range;
      bearing_sum += d.bearing;
    }
  }
  const double rate = static_cast<double>(count) / scans;
  EXPECT_LT(std::abs(rate - 0.5), 3.0 * std::sqrt(0.5 / scans));
  // Uniform on (0, 18.75]: sd 18.75 / sqrt(12); on (-pi, pi]: 2 pi / sqrt(12).
  const double c = static_cast<double>(count);
  EXPECT_LT(std::abs(range_sum / c - 9.375), 3.0 * 18.75 / std::sqrt(12.0 * c));
  EXPECT_LT(std::abs(bearing_sum / c), 3.0 * 2.0 * kPi / std::sqrt(12.0 * c));
}

TEST(FakeDetector, ColourMixUpRate)
{
  // p_c = 0.1: swapped fraction within 3 SE; a swapped colour is never the
  // true one and each of the other three appears about equally often.
  DetectorParams p = test_params();
  p.colour_error_probability = 0.1;
  FakeDetector det(p, {object(0, 8.0, 0.0, BuoyColour::green)}, 11);
  const int n = 100000;
  int swapped = 0;
  int per_colour[5] = {0, 0, 0, 0, 0};
  for (int k = 0; k < n; ++k) {
    const auto ds = det.scan(Pose2{});
    ASSERT_EQ(ds.size(), 1u);
    ASSERT_NE(ds[0].colour, BuoyColour::unknown);
    if (ds[0].colour != BuoyColour::green) {
      ++swapped;
    }
    ++per_colour[static_cast<int>(ds[0].colour)];
  }
  const double frac = static_cast<double>(swapped) / n;
  EXPECT_LT(std::abs(frac - 0.1), 3.0 * std::sqrt(0.1 * 0.9 / n));
  for (int c : {1, 3, 4}) {
    const double share = static_cast<double>(per_colour[c]) / swapped;
    EXPECT_LT(std::abs(share - 1.0 / 3.0), 3.0 * std::sqrt((2.0 / 9.0) / swapped));
  }
}

TEST(FakeDetector, SameSeedSameOutputOtherSeedDiffers)
{
  DetectorParams p = test_params();
  p.range_sigma = 0.1;
  p.bearing_sigma = 0.02;
  p.false_alarm_rate = 0.3;
  p.false_alarm_max_range = 10.0;
  const std::vector<DetectorObject> objs = {object(0, 5.0, 1.0), object(1, 3.0, -2.0)};
  FakeDetector a(p, objs, 7);
  FakeDetector b(p, objs, 7);
  FakeDetector c(p, objs, 8);
  bool differs = false;
  for (int k = 0; k < 100; ++k) {
    const Pose2 pose{0.01 * k, 0.0, 0.001 * k};
    const auto da = a.scan(pose);
    const auto db = b.scan(pose);
    const auto dc = c.scan(pose);
    ASSERT_EQ(da.size(), db.size());
    for (std::size_t i = 0; i < da.size(); ++i) {
      EXPECT_EQ(da[i].range, db[i].range);
      EXPECT_EQ(da[i].bearing, db[i].bearing);
      EXPECT_EQ(da[i].true_id, db[i].true_id);
    }
    if (da.size() != dc.size() || (!da.empty() && da[0].range != dc[0].range)) {
      differs = true;
    }
  }
  EXPECT_TRUE(differs);
}

TEST(FakeDetector, MissesDoNotChangeTheNoiseOfOtherDetections)
{
  // Common random numbers: with misses on, every detection that is kept has
  // exactly the range it has with misses off.
  DetectorParams p = test_params();
  p.range_sigma = 0.1;
  DetectorParams q = p;
  q.miss_probability = 0.5;
  const std::vector<DetectorObject> objs = {object(0, 5.0, 1.0), object(1, 3.0, -2.0)};
  FakeDetector all(p, objs, 4);
  FakeDetector some(q, objs, 4);
  int kept = 0;
  for (int k = 0; k < 200; ++k) {
    const auto da = all.scan(Pose2{});
    for (const auto & d : some.scan(Pose2{})) {
      EXPECT_EQ(d.range, find_id(da, *d.true_id)->range);
      ++kept;
    }
  }
  EXPECT_GT(kept, 100);
  EXPECT_LT(kept, 300);
}

TEST(FakeDetector, RejectsBadParameters)
{
  DetectorParams p = test_params();
  p.miss_probability = 1.5;
  EXPECT_THROW(FakeDetector(p, {}, 1), std::invalid_argument);
  p = test_params();
  p.false_alarm_rate = 0.1;  // max range 0
  EXPECT_THROW(FakeDetector(p, {}, 1), std::invalid_argument);
  p = test_params();
  auto o = object(0, 1.0, 1.0);
  o.camera_range = {};  // one per camera needed
  EXPECT_THROW(FakeDetector(p, {o}, 1), std::invalid_argument);
}

TEST(ScanGrid, EveryFifthOdometryMessage)
{
  // 50 Hz odometry with stamps 4 ms off the 20 ms grid (physics step 4 ms):
  // a 100 ms period gives exactly every fifth message.
  ScanGrid grid(100'000'000);
  std::vector<std::int64_t> scans;
  for (int k = 0; k < 50; ++k) {
    const std::int64_t t = 4'000'000 + 20'000'000LL * k;
    if (grid.due(t)) {
      scans.push_back(t);
    }
  }
  ASSERT_EQ(scans.size(), 10u);
  EXPECT_EQ(scans[0], 4'000'000);  // the first message starts a scan
  for (std::size_t i = 1; i < scans.size(); ++i) {
    EXPECT_EQ(scans[i] - scans[i - 1], 100'000'000);
  }
}

TEST(ScanGrid, OldOrRepeatedStampsAreNotDue)
{
  ScanGrid grid(100'000'000);
  EXPECT_TRUE(grid.due(250'000'000));
  EXPECT_FALSE(grid.due(250'000'000));
  EXPECT_FALSE(grid.due(299'999'999));
  EXPECT_FALSE(grid.due(100'000'000));
  EXPECT_TRUE(grid.due(300'000'000));
  EXPECT_THROW(ScanGrid(0), std::invalid_argument);
}

}  // namespace
