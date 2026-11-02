// Unit tests for the GNSS corruption glue (M2.6b): parameters from asv.yaml
// and a noise profile, and the clean -> corrupted fix (design.md 4.2).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "asv_core/geodesy.hpp"
#include "asv_sim/gnss_corruption.hpp"

namespace
{

using asv_core::GnssCorruptor;
using asv_core::GnssMode;
using asv_core::GnssSchedule;
using asv_msgs::msg::GnssStatus;
using asv_sim::corrupt_gnss;
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
    "  schedule:\n"
    "    - {from: 0, mode: rtk_fixed}\n"
    "    - {from: 60, mode: outage}\n"
    "    - {from: 90.5, mode: standalone}\n");
  const auto p = gnss_error_params(asv_yaml(), profile);
  EXPECT_DOUBLE_EQ(p.rtk_float.white_v, 2.0 * 0.10);
  EXPECT_DOUBLE_EQ(p.rtk_float.slow_v, 0.0);
  EXPECT_DOUBLE_EQ(p.standalone.drift_time, 300.0);  // not scaled
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

}  // namespace
