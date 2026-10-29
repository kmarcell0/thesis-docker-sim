// IMU corruption glue (M2.6a); see imu_corruption.hpp.
#include "asv_sim/imu_corruption.hpp"

#include <stdexcept>
#include <string>

namespace asv_sim
{

namespace
{

// The sensor_specs entry of the one sensor with type imu.
YAML::Node imu_spec(const YAML::Node & asv)
{
  YAML::Node spec;
  int count = 0;
  for (const auto & s : asv["sensors"]) {
    if (s["type"].as<std::string>() == "imu") {
      spec = asv["sensor_specs"][s["spec"].as<std::string>()];
      ++count;
    }
  }
  if (count != 1) {
    throw std::invalid_argument(
            "asv.yaml must have exactly one sensor of type imu, found " + std::to_string(count));
  }
  return spec;
}

asv_core::AxisErrorParams axis_params(
  const YAML::Node & spec, const std::string & sensor, const YAML::Node & mult,
  double drift_time)
{
  asv_core::AxisErrorParams p;
  p.density = spec[sensor + "_noise_density"].as<double>() * mult["white"].as<double>();
  p.turn_on_sigma = spec[sensor + "_bias"].as<double>() * mult["turn_on"].as<double>();
  p.drift_sigma = spec[sensor + "_bias_drift"].as<double>() * mult["drift"].as<double>();
  p.drift_time = drift_time;
  return p;
}

void mark_orientation_unavailable(sensor_msgs::msg::Imu & msg)
{
  msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.0;
  msg.orientation_covariance.fill(0.0);
  msg.orientation_covariance[0] = -1.0;
}

}  // namespace

asv_core::ImuErrorParams imu_error_params(const YAML::Node & asv, const YAML::Node & profile)
{
  const YAML::Node spec = imu_spec(asv);
  const YAML::Node imu = profile["imu"];
  const double drift_time = imu["drift_time"] ?
    imu["drift_time"].as<double>() : spec["bias_drift_time"].as<double>();
  asv_core::ImuErrorParams p;
  p.gyro = axis_params(spec, "gyro", imu["gyro"], drift_time);
  p.accel = axis_params(spec, "accel", imu["accel"], drift_time);
  return p;
}

std::uint64_t profile_seed(const YAML::Node & profile)
{
  return profile["seed"].as<std::uint64_t>();
}

std::optional<ImuMessages> corrupt_imu(
  const sensor_msgs::msg::Imu & clean, asv_core::ImuCorruptor & corruptor)
{
  // Stamp in integer nanoseconds (design.md section 3: time from stamps).
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(clean.header.stamp.sec) * 1'000'000'000LL +
    static_cast<std::int64_t>(clean.header.stamp.nanosec);
  const Eigen::Vector3d gyro(
    clean.angular_velocity.x, clean.angular_velocity.y, clean.angular_velocity.z);
  const Eigen::Vector3d accel(
    clean.linear_acceleration.x, clean.linear_acceleration.y, clean.linear_acceleration.z);

  const auto r = corruptor.process(stamp_ns, gyro, accel);
  if (!r) {
    return std::nullopt;
  }

  // Built field by field from the new values only: nothing of the clean
  // orientation (which holds the true yaw) can reach an output.
  ImuMessages out;
  out.data.header = clean.header;
  out.data.angular_velocity.x = r->gyro.x();
  out.data.angular_velocity.y = r->gyro.y();
  out.data.angular_velocity.z = r->gyro.z();
  out.data.linear_acceleration.x = r->accel.x();
  out.data.linear_acceleration.y = r->accel.y();
  out.data.linear_acceleration.z = r->accel.z();
  out.data.angular_velocity_covariance.fill(0.0);
  out.data.linear_acceleration_covariance.fill(0.0);
  mark_orientation_unavailable(out.data);

  out.bias = out.data;
  out.bias.angular_velocity.x = r->gyro_bias.x();
  out.bias.angular_velocity.y = r->gyro_bias.y();
  out.bias.angular_velocity.z = r->gyro_bias.z();
  out.bias.linear_acceleration.x = r->accel_bias.x();
  out.bias.linear_acceleration.y = r->accel_bias.y();
  out.bias.linear_acceleration.z = r->accel_bias.z();
  return out;
}

}  // namespace asv_sim
