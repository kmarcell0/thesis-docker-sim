// Unit tests for the GNSS corruption glue: parameters from asv.yaml and a
// noise profile, the clean -> corrupted fix (M2.6b, design.md 4.2), velocity
// and heading (M2.6c, design.md 4.2, 4.3).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "asv_core/angles.hpp"
#include "asv_core/geodesy.hpp"
#include "asv_sim/gnss_corruption.hpp"

namespace
{

using asv_core::GnssCorruptor;
using asv_core::GnssMode;
using asv_core::GnssSchedule;
using asv_msgs::msg::GnssStatus;
using asv_sim::corrupt_gnss;
using asv_sim::GnssCorruption;
using asv_sim::gnss_error_params;
using asv_sim::gnss_schedule;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

const std::string kConfig = ASV_CONFIG_DIR;

YAML::Node nominal_profile() {return YAML::LoadFile(kConfig + "/noise_profiles/nominal.yaml");}
YAML::Node asv_yaml() {return YAML::LoadFile(kConfig + "/asv.yaml");}

// The world's reference point (roboboat_course.sdf <spherical_coordinates>).
const asv_core::LocalTangentPlane kWorld({27.367256, -82.451120, 0.0});

// The antenna 100 m east, 100 m north of the world origin, as Gazebo sends it
// (exact since M2.2). Epoch k at 10 Hz from 7.3 s.
const Eigen::Vector3d kAntenna(100.0, 100.0, 0.185);

NavSatFix clean(int k)
{
  NavSatFix m;
  const std::int64_t ns = 7'300'000'000LL + k * 100'000'000LL;
  m.header.stamp.sec = static_cast<std::int32_t>(ns / 1'000'000'000LL);
  m.header.stamp.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000LL);
  m.header.frame_id = "gnss_primary_link";
  m.status.service = NavSatStatus::SERVICE_GPS;
  const auto g = kWorld.to_geo(kAntenna);
  m.latitude = g.lat_deg;
  m.longitude = g.lon_deg;
  m.altitude = g.alt;
  return m;
}

TEST(GnssErrorParams, NominalProfileGivesTheAsvYamlTable)
{
  const auto p = gnss_error_params(asv_yaml(), nominal_profile());
  EXPECT_DOUBLE_EQ(p.rtk_fixed.white_h, 0.005);
  EXPECT_DOUBLE_EQ(p.rtk_fixed.slow_h, 0.006);
  EXPECT_DOUBLE_EQ(p.rtk_fixed.white_v, 0.0094);
  EXPECT_DOUBLE_EQ(p.rtk_fixed.slow_v, 0.0113);
  EXPECT_DOUBLE_EQ(p.rtk_fixed.drift_time, 60.0);
  EXPECT_DOUBLE_EQ(p.rtk_float.white_h, 0.05);
  EXPECT_DOUBLE_EQ(p.rtk_float.slow_h, 0.19);
  EXPECT_DOUBLE_EQ(p.standalone.slow_h, 1.47);
  EXPECT_DOUBLE_EQ(p.standalone.drift_time, 300.0);
  EXPECT_DOUBLE_EQ(p.velocity_white, 0.03);
  // D12 a: 0.1 deg / 1.657 m baseline (asv.yaml antennas at -+(0.76, 0.33))
  // = 0.0603 deg = 1.0533 mrad.
  ASSERT_TRUE(p.heading_white);
  EXPECT_NEAR(*p.heading_white, 0.1 * M_PI / 180.0 / std::hypot(1.52, 0.66), 1e-12);
  EXPECT_NEAR(*p.heading_white, 1.0533e-3, 1e-7);
  // Nominal schedule: RTK fixed for the whole run.
  const auto s = gnss_schedule(nominal_profile());
  ASSERT_EQ(s.entries().size(), 1u);
  EXPECT_EQ(s.mode_at(1e6), GnssMode::rtk_fixed);
}

TEST(GnssErrorParams, MultipliersAndSchedule)
{
  const YAML::Node profile = YAML::Load(
    "gnss:\n"
    "  white: 2.0\n"
    "  slow: 0.0\n"
    "  velocity: 0.5\n"
    "  heading: 3.0\n"
    "  schedule:\n"
    "    - {from: 0, mode: rtk_fixed}\n"
    "    - {from: 60, mode: outage}\n"
    "    - {from: 90.5, mode: standalone}\n");
  const auto p = gnss_error_params(asv_yaml(), profile);
  EXPECT_DOUBLE_EQ(p.rtk_float.white_v, 2.0 * 0.10);
  EXPECT_DOUBLE_EQ(p.rtk_float.slow_v, 0.0);
  EXPECT_DOUBLE_EQ(p.standalone.drift_time, 300.0);  // not scaled
  EXPECT_DOUBLE_EQ(p.velocity_white, 0.5 * 0.03);
  EXPECT_NEAR(*p.heading_white, 3.0 * 1.0533e-3, 1e-6);
  const auto s = gnss_schedule(profile);
  EXPECT_EQ(s.mode_at(59.9), GnssMode::rtk_fixed);
  EXPECT_EQ(s.mode_at(60.0), GnssMode::outage);
  EXPECT_EQ(s.mode_at(90.5), GnssMode::standalone);
}

TEST(GnssErrorParams, BadProfileOrNoGnssThrows)
{
  EXPECT_THROW(
    gnss_schedule(YAML::Load("gnss: {schedule: [{from: 0, mode: rtk}]}")),
    std::invalid_argument);
  EXPECT_THROW(
    gnss_schedule(YAML::Load(
      "gnss: {schedule: [{from: 0, mode: rtk_fixed}, {from: 0, mode: outage}]}")),
    std::invalid_argument);
  EXPECT_THROW(
    gnss_error_params(asv_yaml(), YAML::Load("gnss: {white: 1.0}")), YAML::Exception);
  YAML::Node no_gnss = asv_yaml();
  no_gnss["sensors"] = YAML::Load("[{name: lidar, type: lidar, spec: livox_mid360}]");
  EXPECT_THROW(gnss_error_params(no_gnss, nominal_profile()), std::invalid_argument);
}

// The corrupted fix, converted to world ENU with the geodesy helper, is the
// true antenna position plus exactly the error GnssCorruptor drew (a twin
// with the same seed gives it). Standalone, so the error is metres.
TEST(CorruptGnss, FixIsTruthPlusTheErrorInWorldEnu)
{
  const auto p = gnss_error_params(asv_yaml(), nominal_profile());
  GnssCorruptor c(p, GnssSchedule({{0.0, GnssMode::standalone}}), 1);
  GnssCorruptor twin(p, GnssSchedule({{0.0, GnssMode::standalone}}), 1);
  double largest = 0.0;
  for (int k = 0; k < 100; ++k) {
    const auto out = corrupt_gnss(clean(k), c);
    const auto ep = twin.process(7'300'000'000LL + k * 100'000'000LL);
    ASSERT_TRUE(out && out->fix && ep && ep->error);
    const Eigen::Vector3d enu =
      kWorld.to_enu({out->fix->latitude, out->fix->longitude, out->fix->altitude});
    largest = std::max(largest, (enu - kAntenna - *ep->error).norm());
    EXPECT_EQ(out->slow_error->vector.x, ep->slow_error.x());
  }
  EXPECT_LT(largest, 1e-4);  // m; the axis tilt of the local frame is ~ 4e-5 m here
}

TEST(CorruptGnss, CovarianceStatusAndHeaderPerMode)
{
  const auto p = gnss_error_params(asv_yaml(), nominal_profile());
  struct Case {GnssMode mode; std::uint8_t status; double sigma_h; bool lost;};
  const Case cases[] = {
    {GnssMode::rtk_fixed, NavSatStatus::STATUS_GBAS_FIX, std::hypot(0.005, 0.006), false},
    {GnssMode::rtk_float, NavSatStatus::STATUS_GBAS_FIX, std::hypot(0.05, 0.19), false},
    {GnssMode::standalone, NavSatStatus::STATUS_FIX, std::hypot(0.3, 1.47), true}};
  for (const auto & tc : cases) {
    GnssCorruptor c(p, GnssSchedule({{0.0, tc.mode}}), 1);
    const NavSatFix in = clean(0);
    const auto out = corrupt_gnss(in, c);
    ASSERT_TRUE(out && out->fix);
    EXPECT_EQ(out->fix->header, in.header);
    EXPECT_EQ(out->fix->status.status, tc.status);
    EXPECT_EQ(out->fix->status.service, NavSatStatus::SERVICE_GPS);
    EXPECT_EQ(out->fix->position_covariance_type, NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
    EXPECT_DOUBLE_EQ(out->fix->position_covariance[0], tc.sigma_h * tc.sigma_h);
    EXPECT_DOUBLE_EQ(out->fix->position_covariance[4], tc.sigma_h * tc.sigma_h);
    EXPECT_GT(out->fix->position_covariance[8], out->fix->position_covariance[0]);
    for (const int off : {1, 2, 3, 5, 6, 7}) {EXPECT_EQ(out->fix->position_covariance[off], 0.0);}
    EXPECT_EQ(out->status.header.stamp, in.header.stamp);
    EXPECT_EQ(out->status.position_mode, static_cast<std::uint8_t>(tc.mode));
    EXPECT_FALSE(out->status.sky_view_outage);
    EXPECT_EQ(out->status.corrections_lost, tc.lost);
    EXPECT_FALSE(out->status.heading_available);
    EXPECT_EQ(out->slow_error->header.frame_id, "world");
  }
}

TEST(CorruptGnss, OutageGivesStatusOnly)
{
  const auto p = gnss_error_params(asv_yaml(), nominal_profile());
  GnssCorruptor c(p, GnssSchedule({{0.0, GnssMode::outage}}), 1);
  const auto out = corrupt_gnss(clean(0), c);
  ASSERT_TRUE(out);
  EXPECT_FALSE(out->fix);
  EXPECT_FALSE(out->slow_error);
  EXPECT_EQ(out->status.position_mode, GnssStatus::MODE_NO_FIX);
  EXPECT_TRUE(out->status.sky_view_outage);
  EXPECT_FALSE(out->status.heading_available);
}

TEST(CorruptGnss, FirstFixPublishedRepeatedStampDropped)
{
  GnssCorruptor c(gnss_error_params(asv_yaml(), nominal_profile()),
    gnss_schedule(nominal_profile()), 1);
  EXPECT_TRUE(corrupt_gnss(clean(0), c));
  EXPECT_FALSE(corrupt_gnss(clean(0), c));
  EXPECT_TRUE(corrupt_gnss(clean(1), c));
}

// ---- velocity and heading (M2.6c) -------------------------------------------

// Profile with every size multiplier 0: the outputs are the truth exactly.
YAML::Node silent_profile()
{
  return YAML::Load(
    "gnss: {white: 0.0, slow: 0.0, velocity: 0.0, heading: 0.0,"
    " schedule: [{from: 0, mode: rtk_fixed}]}");
}

// The antenna on a straight line at (1.2, -0.5, 0) m/s from kAntenna.
NavSatFix moving(int k)
{
  NavSatFix m = clean(k);
  const Eigen::Vector3d p = kAntenna + 0.1 * k * Eigen::Vector3d(1.2, -0.5, 0.0);
  const auto g = kWorld.to_geo(p);
  m.latitude = g.lat_deg;
  m.longitude = g.lon_deg;
  m.altitude = g.alt;
  return m;
}

nav_msgs::msg::Odometry odom(std::int64_t ns, double psi)
{
  nav_msgs::msg::Odometry o;
  o.header.stamp.sec = static_cast<std::int32_t>(ns / 1'000'000'000LL);
  o.header.stamp.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000LL);
  o.pose.pose.orientation.w = std::cos(psi / 2.0);
  o.pose.pose.orientation.z = std::sin(psi / 2.0);
  return o;
}

std::int64_t ns_of(int k) {return 7'300'000'000LL + k * 100'000'000LL;}

// Velocity: one epoch late, stamped with the middle epoch, equal to the true
// velocity without noise; covariance layout.
TEST(GnssCorruption, VelocityIsTheCentralDifferenceOneEpochLate)
{
  const auto p = gnss_error_params(asv_yaml(), silent_profile());
  GnssCorruption c(p, gnss_schedule(silent_profile()), 1);
  EXPECT_FALSE(c.on_fix(moving(0)).velocity);
  EXPECT_FALSE(c.on_fix(moving(1)).velocity);  // needs fix 2 for epoch 1
  for (int k = 2; k < 20; ++k) {
    const auto out = c.on_fix(moving(k));
    ASSERT_TRUE(out.velocity) << k;
    const auto & v = *out.velocity;
    EXPECT_EQ(v.header.stamp, moving(k - 1).header.stamp);
    EXPECT_EQ(v.header.frame_id, "world");
    // The velocity is in an ENU frame centred on the antenna, tilted from
    // the world's by d / R_E = 141 m / 6.4e6 m = 2.2e-5 rad: up to 3e-5 m/s
    // at 1.3 m/s (0.1 % of sigma_v).
    EXPECT_NEAR(v.twist.twist.linear.x, 1.2, 5e-5);
    EXPECT_NEAR(v.twist.twist.linear.y, -0.5, 5e-5);
    EXPECT_NEAR(v.twist.twist.linear.z, 0.0, 5e-5);
  }
  // With noise: linear diag sigma_v^2, angular diag -1, rest 0.
  GnssCorruption n(gnss_error_params(asv_yaml(), nominal_profile()),
    gnss_schedule(nominal_profile()), 1);
  for (int k = 0; k < 2; ++k) {n.on_fix(moving(k));}
  const auto out = n.on_fix(moving(2));
  ASSERT_TRUE(out.velocity);
  const auto & cov = out.velocity->twist.covariance;
  for (std::size_t i = 0; i < 36; ++i) {
    const double expect = (i == 0 || i == 7 || i == 14) ? 0.03 * 0.03 :
      (i == 21 || i == 28 || i == 35) ? -1.0 : 0.0;
    EXPECT_DOUBLE_EQ(cov[i], expect) << i;
  }
  EXPECT_NE(out.velocity->twist.twist.linear.x, 1.2);
}

// Heading = pi/2 - psi without noise; it waits for odometry up to its stamp;
// the message fields.
TEST(GnssCorruption, HeadingWaitsForTheYawAndIsTheAzimuth)
{
  const auto p = gnss_error_params(asv_yaml(), silent_profile());
  GnssCorruption c(p, gnss_schedule(silent_profile()), 1);
  // Odometry at 50 Hz up to just before epoch 0: the heading must wait.
  for (std::int64_t t = ns_of(0) - 100'000'000; t < ns_of(0); t += 20'000'000) {
    c.on_odom(odom(t, 2.0));
  }
  const auto out = c.on_fix(clean(0));
  EXPECT_TRUE(out.headings.empty());
  EXPECT_TRUE(out.position->status.heading_available);
  // The odometry sample at the stamp arrives: the heading is ready.
  const auto ready = c.on_odom(odom(ns_of(0), 3.0));
  ASSERT_EQ(ready.size(), 1u);
  const auto & h = ready[0];
  EXPECT_EQ(h.header.stamp, clean(0).header.stamp);
  EXPECT_EQ(h.header.frame_id, "base_link");
  EXPECT_NEAR(h.azimuth, asv_core::azimuth_from_yaw(3.0), 1e-12);
  EXPECT_NEAR(h.azimuth, 2.0 * M_PI + M_PI / 2.0 - 3.0, 1e-12);
  EXPECT_TRUE(h.baseline_fixed);
  EXPECT_DOUBLE_EQ(h.azimuth_std, 0.0);  // silent profile
  // Odometry already there: the heading comes with the fix.
  c.on_odom(odom(ns_of(1), -1.0));
  const auto next = c.on_fix(clean(1));
  ASSERT_EQ(next.headings.size(), 1u);
  EXPECT_NEAR(next.headings[0].azimuth, M_PI / 2.0 + 1.0, 1e-12);
  EXPECT_EQ(c.dropped_headings(), 0u);
}

// Nominal noise: heading error spread sigma_alpha (5 %), velocity error
// spread sigma_v (5 %), on 4000 epochs.
TEST(GnssCorruption, NominalSpreads)
{
  const auto p = gnss_error_params(asv_yaml(), nominal_profile());
  GnssCorruption c(p, gnss_schedule(nominal_profile()), 7);
  std::vector<double> dh, dv;
  for (int k = 0; k < 4000; ++k) {
    c.on_odom(odom(ns_of(k), 0.01 * k));
    const auto out = c.on_fix(moving(k));
    ASSERT_EQ(out.headings.size(), 1u);
    dh.push_back(asv_core::wrap_angle(
        out.headings[0].azimuth - asv_core::azimuth_from_yaw(0.01 * k)));
    EXPECT_DOUBLE_EQ(out.headings[0].azimuth_std, *p.heading_white);
    if (out.velocity) {dv.push_back(out.velocity->twist.twist.linear.x - 1.2);}
  }
  auto sd = [](const std::vector<double> & x) {
      double m = 0.0, q = 0.0;
      for (double v : x) {m += v;}
      m /= static_cast<double>(x.size());
      for (double v : x) {q += (v - m) * (v - m);}
      return std::sqrt(q / static_cast<double>(x.size() - 1));
    };
  EXPECT_NEAR(sd(dh) / *p.heading_white, 1.0, 0.05);
  EXPECT_NEAR(sd(dv) / 0.03, 1.0, 0.05);
}

// Sky-view outage: no velocity, no heading, heading_available false.
// Standalone (corrections lost): heading still there.
TEST(GnssCorruption, OutageStopsBothStandaloneKeepsHeading)
{
  const YAML::Node profile = YAML::Load(
    "gnss: {white: 1.0, slow: 1.0, velocity: 1.0, heading: 1.0, schedule: ["
    "{from: 0, mode: standalone}, {from: 1.0, mode: outage}, {from: 2.0, mode: rtk_fixed}]}");
  GnssCorruption c(gnss_error_params(asv_yaml(), profile), gnss_schedule(profile), 1);
  for (int k = 0; k < 30; ++k) {
    c.on_odom(odom(ns_of(k), 0.5));
    const auto out = c.on_fix(moving(k));
    const bool outage = k >= 10 && k < 20;
    EXPECT_EQ(out.headings.size(), outage ? 0u : 1u) << k;
    EXPECT_EQ(out.position->status.heading_available, !outage) << k;
    // The velocity given out at fix k belongs to epoch k-1.
    const bool prev_outage = k - 1 >= 10 && k - 1 < 20;
    EXPECT_EQ(out.velocity.has_value(), k >= 2 && !prev_outage) << k;
  }
}

// Odometry that never covers the epochs: at most 50 headings wait, older
// ones are dropped and counted; a stamp older than the kept yaw is dropped.
TEST(GnssCorruption, HeadingsWithoutYawAreDropped)
{
  GnssCorruption c(gnss_error_params(asv_yaml(), nominal_profile()),
    gnss_schedule(nominal_profile()), 1);
  for (int k = 0; k < 60; ++k) {EXPECT_TRUE(c.on_fix(clean(k)).headings.empty());}
  EXPECT_EQ(c.dropped_headings(), 10u);  // 60 epochs, 50 may wait
  // Odometry 3 s later: the waiting epochs are older than its history.
  EXPECT_TRUE(c.on_odom(odom(ns_of(90), 0.0)).empty());
  EXPECT_EQ(c.dropped_headings(), 60u);
}

}  // namespace
