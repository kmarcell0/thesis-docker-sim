// IMU corruptor (M2.6a); see imu_corruptor.hpp for the equations.
#include "asv_core/imu_corruptor.hpp"

#include <string>

namespace asv_core
{

namespace
{

// Name of one axis, e.g. "imu.gyro_z"; Axis appends the part (".white").
std::string axis_name(const std::string & sensor, int axis)
{
  return "imu." + sensor + "_" + "xyz"[axis];
}

}  // namespace

ImuCorruptor::Axis::Axis(
  const AxisErrorParams & p, std::uint64_t seed, const std::string & name)
: white(p.density, seed, name + ".white"),
  turn_on(p.turn_on_sigma, seed, name + ".turn_on"),
  drift(p.drift_sigma, p.drift_time, seed, name + ".drift")
{
}

ImuCorruptor::ImuCorruptor(const ImuErrorParams & params, std::uint64_t global_seed)
: gyro_{Axis(params.gyro, global_seed, axis_name("gyro", 0)),
    Axis(params.gyro, global_seed, axis_name("gyro", 1)),
    Axis(params.gyro, global_seed, axis_name("gyro", 2))},
  accel_{Axis(params.accel, global_seed, axis_name("accel", 0)),
    Axis(params.accel, global_seed, axis_name("accel", 1)),
    Axis(params.accel, global_seed, axis_name("accel", 2))}
{
}

std::optional<CorruptedImu> ImuCorruptor::process(
  std::int64_t stamp_ns, const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel)
{
  if (!last_stamp_ns_ || stamp_ns <= *last_stamp_ns_) {
    // First sample: remember its stamp. Old or repeated stamp: ignore it.
    if (!last_stamp_ns_) {
      last_stamp_ns_ = stamp_ns;
    }
    return std::nullopt;
  }
  // Integer nanoseconds, so dt has no rounding error even at large stamps.
  const double dt = static_cast<double>(stamp_ns - *last_stamp_ns_) * 1e-9;
  last_stamp_ns_ = stamp_ns;

  CorruptedImu out;
  corrupt(gyro_, dt, gyro, out.gyro, out.gyro_bias);
  corrupt(accel_, dt, accel, out.accel, out.accel_bias);
  return out;
}

void ImuCorruptor::corrupt(
  std::array<Axis, 3> & axes, double dt, const Eigen::Vector3d & truth,
  Eigen::Vector3d & measured, Eigen::Vector3d & bias)
{
  for (int i = 0; i < 3; ++i) {
    Axis & a = axes[static_cast<std::size_t>(i)];
    // b0 + b(k): the Gauss-Markov part moves on by dt first.
    bias[i] = a.turn_on.value() + a.drift.step(dt);
    measured[i] = truth[i] + bias[i] + a.white.sample(dt);
  }
}

}  // namespace asv_core
