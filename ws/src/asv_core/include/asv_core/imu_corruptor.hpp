// IMU corruptor (M2.6a): turns one clean IMU sample into a realistic one,
// ROS-free. Thesis: "Noise Models", "Vehicle and Sensor Models"
// (design.md section 4.1).
//
// For every axis i of the gyro and of the accelerometer:
//   measured_i = true_i + b0_i + b_i(k) + n_i(k)
//   n_i(k)  white noise, sigma = eta / sqrt(dt_k)            (WhiteNoise)
//   b0_i    turn-on bias ~ N(0, sigma0^2), one draw per run   (TurnOnBias)
//   b_i(k)  wandering bias, Gauss-Markov (sigma_ss, T_c)      (GaussMarkov)
// dt_k comes from the sample stamps (design.md section 3). Each of the
// 3 axes x 2 sensors x 3 parts = 18 sources has its own random stream, named
// e.g. "imu.gyro_z.white" (noise.hpp). Scale-factor and misalignment errors
// are left out (D11 question 8 a, 2026-10-29).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "asv_core/noise.hpp"

namespace asv_core
{

// Error model of one sensor (gyro or accelerometer); the same for its three
// axes. Units: gyro rad/s, accelerometer m/s^2.
struct AxisErrorParams
{
  double density = 0.0;        // eta, white noise density (unit/sqrt(Hz))
  double turn_on_sigma = 0.0;  // sigma0 of the turn-on bias
  double drift_sigma = 0.0;    // sigma_ss of the Gauss-Markov bias
  double drift_time = 1.0;     // T_c of the Gauss-Markov bias (s), > 0
};

struct ImuErrorParams
{
  AxisErrorParams gyro;
  AxisErrorParams accel;
};

// One corrupted sample and the true total bias b0 + b(k) that is in it, so
// the evaluation can compare the filter's bias estimate with the truth.
struct CorruptedImu
{
  Eigen::Vector3d gyro;        // rad/s
  Eigen::Vector3d accel;       // m/s^2
  Eigen::Vector3d gyro_bias;   // rad/s, b0 + b(k)
  Eigen::Vector3d accel_bias;  // m/s^2, b0 + b(k)
};

class ImuCorruptor
{
public:
  ImuCorruptor(const ImuErrorParams & params, std::uint64_t global_seed);

  // Corrupts the sample taken at stamp_ns (ns). Returns nothing for the
  // first sample (no dt yet) and for a stamp that is not later than the last
  // used one (dt <= 0); such a sample changes no state.
  std::optional<CorruptedImu> process(
    std::int64_t stamp_ns, const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel);

private:
  // The three error parts of one axis.
  struct Axis
  {
    Axis(const AxisErrorParams & p, std::uint64_t seed, const std::string & name);
    WhiteNoise white;
    TurnOnBias turn_on;
    GaussMarkov drift;
  };

  // Advances the 3 axes of one sensor by dt; writes measured and bias.
  static void corrupt(
    std::array<Axis, 3> & axes, double dt, const Eigen::Vector3d & truth,
    Eigen::Vector3d & measured, Eigen::Vector3d & bias);

  std::array<Axis, 3> gyro_;
  std::array<Axis, 3> accel_;
  std::optional<std::int64_t> last_stamp_ns_;
};

}  // namespace asv_core
