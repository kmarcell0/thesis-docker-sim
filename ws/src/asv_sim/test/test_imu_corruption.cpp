// Unit tests for the IMU corruption glue (M2.6a): parameters from asv.yaml
// and a noise profile, and the clean -> corrupted message (design.md 4.1).
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "asv_sim/imu_corruption.hpp"

namespace
{

using asv_core::ImuCorruptor;
using asv_sim::corrupt_imu;
using asv_sim::imu_error_params;
using sensor_msgs::msg::Imu;

const std::string kConfig = ASV_CONFIG_DIR;

// A clean message as Gazebo sends it (M2.2: exact values, true orientation).
// Stamps on the IMU's step pattern 8, 4, 4, 4 ms, starting at 12.000 s.
Imu clean(std::size_t i, double yaw)
{
  Imu m;
  const long ns = 12'000'000'000L + static_cast<long>(i / 4) * 20'000'000L +
    std::vector<long>{0, 8'000'000, 12'000'000, 16'000'000}[i % 4];
  m.header.stamp.sec = static_cast<int>(ns / 1'000'000'000L);
  m.header.stamp.nanosec = static_cast<unsigned>(ns % 1'000'000'000L);
  m.header.frame_id = "imu_link";
  m.orientation.z = std::sin(yaw / 2.0);
  m.orientation.w = std::cos(yaw / 2.0);
  m.orientation_covariance.fill(0.0);
  m.angular_velocity.z = 0.1;
  m.linear_acceleration.x = 0.2;
  m.linear_acceleration.z = 9.80665;
  return m;
}

YAML::Node nominal_profile() {return YAML::LoadFile(kConfig + "/noise_profiles/nominal.yaml");}
YAML::Node asv_yaml() {return YAML::LoadFile(kConfig + "/asv.yaml");}

TEST(ImuErrorParams, NominalProfileGivesTheAsvYamlValues)
{
  const auto p = imu_error_params(asv_yaml(), nominal_profile());
  EXPECT_DOUBLE_EQ(p.gyro.density, 6.632e-5);   // [DS]
  EXPECT_DOUBLE_EQ(p.accel.density, 1.079e-3);  // [DS] +-32 g (D11 q9 a)
  EXPECT_DOUBLE_EQ(p.gyro.turn_on_sigma, 0.0017);
  EXPECT_DOUBLE_EQ(p.accel.turn_on_sigma, 0.02);
  EXPECT_DOUBLE_EQ(p.gyro.drift_sigma, 0.0002);
  EXPECT_DOUBLE_EQ(p.accel.drift_sigma, 0.005);
  EXPECT_DOUBLE_EQ(p.gyro.drift_time, 300.0);
  EXPECT_DOUBLE_EQ(p.accel.drift_time, 300.0);
  EXPECT_EQ(asv_sim::profile_seed(nominal_profile()), 1u);
}

TEST(ImuErrorParams, MultipliersAndDriftTimeOverride)
{
  const YAML::Node profile = YAML::Load(
    "seed: 5\n"
    "imu:\n"
    "  gyro: {white: 2.0, turn_on: 0.0, drift: 0.5}\n"
    "  accel: {white: 0.0, turn_on: 1.0, drift: 3.0}\n"
    "  drift_time: 60\n");
  const auto p = imu_error_params(asv_yaml(), profile);
  EXPECT_DOUBLE_EQ(p.gyro.density, 2.0 * 6.632e-5);
  EXPECT_DOUBLE_EQ(p.gyro.turn_on_sigma, 0.0);
  EXPECT_DOUBLE_EQ(p.gyro.drift_sigma, 0.5 * 0.0002);
  EXPECT_DOUBLE_EQ(p.accel.density, 0.0);
  EXPECT_DOUBLE_EQ(p.accel.turn_on_sigma, 0.02);
  EXPECT_DOUBLE_EQ(p.accel.drift_sigma, 3.0 * 0.005);
  EXPECT_DOUBLE_EQ(p.gyro.drift_time, 60.0);
  EXPECT_EQ(asv_sim::profile_seed(profile), 5u);
}

TEST(ImuErrorParams, MissingKeyOrNoImuThrows)
{
  const YAML::Node missing = YAML::Load(
    "imu:\n"
    "  gyro: {white: 1.0, turn_on: 1.0, drift: 1.0}\n"
    "  accel: {white: 1.0, turn_on: 1.0}\n");
  EXPECT_THROW(imu_error_params(asv_yaml(), missing), YAML::Exception);
  YAML::Node no_imu = asv_yaml();
  no_imu["sensors"] = YAML::Load("[{name: lidar, type: lidar, spec: livox_mid360}]");
  EXPECT_THROW(imu_error_params(no_imu, nominal_profile()), std::invalid_argument);
}

TEST(CorruptImu, FirstMessageGivesNothing)
{
  ImuCorruptor c(imu_error_params(asv_yaml(), nominal_profile()), 1);
  EXPECT_FALSE(corrupt_imu(clean(0, 0.3), c).has_value());
  EXPECT_TRUE(corrupt_imu(clean(1, 0.3), c).has_value());
}

TEST(CorruptImu, OrientationUnavailableHeaderCopied)
{
  ImuCorruptor c(imu_error_params(asv_yaml(), nominal_profile()), 1);
  corrupt_imu(clean(0, 0.3), c);
  const Imu in = clean(1, 0.3);
  const auto out = corrupt_imu(in, c);
  ASSERT_TRUE(out.has_value());
  for (const Imu * m : {&out->data, &out->bias}) {
    EXPECT_EQ(m->header, in.header);
    EXPECT_EQ(m->orientation_covariance[0], -1.0);
    EXPECT_EQ(m->orientation.x, 0.0);
    EXPECT_EQ(m->orientation.y, 0.0);
    EXPECT_EQ(m->orientation.z, 0.0);
    EXPECT_EQ(m->orientation.w, 0.0);
    for (std::size_t k = 0; k < 9; ++k) {
      EXPECT_EQ(m->angular_velocity_covariance[k], 0.0);
      EXPECT_EQ(m->linear_acceleration_covariance[k], 0.0);
    }
  }
}

// The proof that no output carries the true yaw: two runs that differ only
// in the clean orientation (yaw 0.3 rad vs. a yaw that changes every sample)
// give bit-identical outputs, so the outputs cannot depend on the yaw.
TEST(CorruptImu, TrueYawNeverReachesAnOutput)
{
  const auto p = imu_error_params(asv_yaml(), nominal_profile());
  ImuCorruptor a(p, 1), b(p, 1);
  for (std::size_t i = 0; i < 2000; ++i) {
    const auto ra = corrupt_imu(clean(i, 0.3), a);
    const auto rb = corrupt_imu(clean(i, -3.0 + 0.003 * static_cast<double>(i)), b);
    ASSERT_EQ(ra.has_value(), rb.has_value());
    if (ra) {
      EXPECT_EQ(ra->data, rb->data);
      EXPECT_EQ(ra->bias, rb->bias);
    }
  }
}

// The messages hold exactly what ImuCorruptor computed for that stamp.
TEST(CorruptImu, MessagesHoldTheCorruptorValues)
{
  const auto p = imu_error_params(asv_yaml(), nominal_profile());
  ImuCorruptor a(p, 1), b(p, 1);
  for (std::size_t i = 0; i < 100; ++i) {
    const Imu in = clean(i, 0.3);
    const auto msgs = corrupt_imu(in, a);
    const std::int64_t ns = static_cast<std::int64_t>(in.header.stamp.sec) * 1'000'000'000LL +
      in.header.stamp.nanosec;
    const auto r = b.process(
      ns, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d(0.2, 0.0, 9.80665));
    ASSERT_EQ(msgs.has_value(), r.has_value());
    if (r) {
      EXPECT_EQ(msgs->data.angular_velocity.z, r->gyro.z());
      EXPECT_EQ(msgs->data.linear_acceleration.x, r->accel.x());
      EXPECT_EQ(msgs->bias.angular_velocity.z, r->gyro_bias.z());
      EXPECT_EQ(msgs->bias.linear_acceleration.z, r->accel_bias.z());
    }
  }
}

}  // namespace
