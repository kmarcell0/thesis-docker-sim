// Glue between the corruptor node and asv_core::ImuCorruptor (M2.6a): reads
// the IMU error parameters from asv.yaml and a noise profile, and turns one
// clean sensor_msgs/Imu into the corrupted message and the true-bias message.
// No node here, so it is unit tested (design.md section 4.1).
#pragma once

#include <cstdint>
#include <optional>

#include "asv_core/imu_corruptor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "yaml-cpp/yaml.h"

namespace asv_sim
{

// Error parameters of the one sensor with type imu in asv.yaml: each
// nominal value of its sensor_specs entry times the profile's multiplier
// (profile imu.gyro / imu.accel: white, turn_on, drift); T_c is
// bias_drift_time unless the profile sets imu.drift_time. A missing key
// throws (YAML::Exception), no IMU or more than one throws
// std::invalid_argument.
asv_core::ImuErrorParams imu_error_params(const YAML::Node & asv, const YAML::Node & profile);

// Global seed of the profile (key seed).
std::uint64_t profile_seed(const YAML::Node & profile);

struct ImuMessages
{
  // /asv/sensors/imu/data: noisy gyro and accelerometer, same header as the
  // clean message. Orientation not available (D11): quaternion all zero and
  // orientation_covariance[0] = -1. The other covariances all zero
  // ("unknown"): the filter takes its noise values from its own config.
  sensor_msgs::msg::Imu data;
  // /asv/ground_truth/imu_bias: the true total bias b0 + b(k) in the same
  // fields (angular_velocity rad/s, linear_acceleration m/s^2), same header.
  sensor_msgs::msg::Imu bias;
};

// Corrupts one clean message. Nothing for the first message and for a stamp
// not later than the previous one (ImuCorruptor::process()).
std::optional<ImuMessages> corrupt_imu(
  const sensor_msgs::msg::Imu & clean, asv_core::ImuCorruptor & corruptor);

}  // namespace asv_sim
