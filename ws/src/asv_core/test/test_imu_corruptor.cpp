// Unit tests for the IMU corruptor (M2.6a): design.md 4.1.
// The statistics checks use fixed seeds and the 5 % tolerance of design.md 4.5.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "allan.hpp"
#include "asv_core/imu_corruptor.hpp"

namespace
{

using asv_core::AxisErrorParams;
using asv_core::CorruptedImu;
using asv_core::ImuCorruptor;
using asv_core::ImuErrorParams;
using asv_core_test::mean;
using asv_core_test::std_dev;
using Eigen::Vector3d;

constexpr double kStatTol = 0.05;  // relative, design.md 4.5

// Nominal IMU values [REPO asv.yaml sensor_specs, M2.6a]: densities [DS],
// biases and correlation time [EST].
ImuErrorParams nominal()
{
  ImuErrorParams p;
  p.gyro = {6.632e-5, 0.0017, 0.0002, 300.0};  // rad/s
  p.accel = {1.079e-3, 0.02, 0.005, 300.0};    // m/s^2
  return p;
}

// Stamps on the IMU's real step pattern 8, 4, 4, 4 ms (design.md 3, rule 4),
// starting at a large stamp so integer-nanosecond dt is exercised.
std::int64_t stamp_ns(std::size_t i)
{
  const std::int64_t start = 1'000'000'000'000LL;  // 1000 s
  const auto block = static_cast<std::int64_t>(i / 4) * 20'000'000LL;
  const std::int64_t within[4] = {0, 8'000'000, 12'000'000, 16'000'000};
  return start + block + within[i % 4];
}

// A changing "true" motion, so the test sees errors, not just the noise of 0.
Vector3d true_gyro(std::size_t i) {return Vector3d(0.1, -0.2, 0.3 * std::sin(1e-3 * i));}
Vector3d true_accel(std::size_t i) {return Vector3d(0.5 * std::cos(1e-3 * i), 0.0, 9.80665);}

// Runs n samples (the first gives no output) and returns the outputs.
std::vector<CorruptedImu> run(ImuCorruptor & c, std::size_t n)
{
  std::vector<CorruptedImu> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (auto r = c.process(stamp_ns(i), true_gyro(i), true_accel(i))) {
      out.push_back(*r);
    }
  }
  return out;
}

bool same(const CorruptedImu & a, const CorruptedImu & b)
{
  return a.gyro == b.gyro && a.accel == b.accel && a.gyro_bias == b.gyro_bias &&
         a.accel_bias == b.accel_bias;
}

TEST(ImuCorruptor, FirstSampleGivesNothing)
{
  ImuCorruptor c(nominal(), 7);
  EXPECT_FALSE(c.process(stamp_ns(0), true_gyro(0), true_accel(0)).has_value());
  EXPECT_TRUE(c.process(stamp_ns(1), true_gyro(1), true_accel(1)).has_value());
}

TEST(ImuCorruptor, SameSeedSameOutputOtherSeedOtherOutput)
{
  ImuCorruptor a(nominal(), 7), b(nominal(), 7), c(nominal(), 8);
  const auto ra = run(a, 1000), rb = run(b, 1000), rc = run(c, 1000);
  ASSERT_EQ(ra.size(), 999u);
  for (std::size_t k = 0; k < ra.size(); ++k) {
    EXPECT_TRUE(same(ra[k], rb[k]));
  }
  EXPECT_NE(ra[0].gyro, rc[0].gyro);
}

// A repeated or older stamp is dropped and changes no state: the outputs
// afterwards equal those of a corruptor that never saw it.
TEST(ImuCorruptor, NonIncreasingStampIsDroppedWithoutSideEffects)
{
  ImuCorruptor a(nominal(), 7), b(nominal(), 7);
  for (std::size_t i = 0; i < 10; ++i) {
    a.process(stamp_ns(i), true_gyro(i), true_accel(i));
    b.process(stamp_ns(i), true_gyro(i), true_accel(i));
  }
  EXPECT_FALSE(b.process(stamp_ns(9), true_gyro(9), true_accel(9)).has_value());
  EXPECT_FALSE(b.process(stamp_ns(3), true_gyro(3), true_accel(3)).has_value());
  for (std::size_t i = 10; i < 20; ++i) {
    const auto ra = a.process(stamp_ns(i), true_gyro(i), true_accel(i));
    const auto rb = b.process(stamp_ns(i), true_gyro(i), true_accel(i));
    ASSERT_TRUE(ra && rb);
    EXPECT_TRUE(same(*ra, *rb));
  }
}

TEST(ImuCorruptor, ZeroParametersGiveTheTruthExactly)
{
  ImuErrorParams p;  // all densities and sigmas 0
  ImuCorruptor c(p, 7);
  for (std::size_t i = 0; i < 100; ++i) {
    if (auto r = c.process(stamp_ns(i), true_gyro(i), true_accel(i))) {
      EXPECT_EQ(r->gyro, true_gyro(i));
      EXPECT_EQ(r->accel, true_accel(i));
      EXPECT_EQ(r->gyro_bias, Vector3d::Zero());
      EXPECT_EQ(r->accel_bias, Vector3d::Zero());
    }
  }
}

// Common random numbers: changing the accelerometer settings leaves the gyro
// numbers unchanged (each part of each axis has its own stream).
TEST(ImuCorruptor, AccelSettingsDoNotChangeGyroNumbers)
{
  ImuErrorParams other = nominal();
  other.accel = {0.0, 0.0, 0.0, 10.0};
  ImuCorruptor a(nominal(), 7), b(other, 7);
  const auto ra = run(a, 1000), rb = run(b, 1000);
  for (std::size_t k = 0; k < ra.size(); ++k) {
    EXPECT_EQ(ra[k].gyro, rb[k].gyro);
    EXPECT_EQ(ra[k].gyro_bias, rb[k].gyro_bias);
  }
}

// Error minus the logged true bias is the white noise: mean 0 and, divided by
// sigma = eta/sqrt(dt) of its own step, spread 1. Also the raw spread on the
// 4 ms steps (gyro 6.632e-5/sqrt(0.004) = 1.049e-3 rad/s). 1 h at 200 Hz.
TEST(ImuCorruptor, ErrorMinusBiasIsWhiteNoiseOfTheProfile)
{
  const ImuErrorParams p = nominal();
  ImuCorruptor c(p, 11);
  const std::size_t n = 720'000;
  std::vector<std::vector<double>> norm(6), raw4(6);
  c.process(stamp_ns(0), true_gyro(0), true_accel(0));  // first: no output
  for (std::size_t i = 1; i <= n; ++i) {
    const auto r = c.process(stamp_ns(i), true_gyro(i), true_accel(i));
    ASSERT_TRUE(r.has_value());
    const double dt = static_cast<double>(stamp_ns(i) - stamp_ns(i - 1)) * 1e-9;
    for (int a = 0; a < 3; ++a) {
      const double eg = r->gyro[a] - true_gyro(i)[a] - r->gyro_bias[a];
      const double ea = r->accel[a] - true_accel(i)[a] - r->accel_bias[a];
      norm[a].push_back(eg / (p.gyro.density / std::sqrt(dt)));
      norm[3 + a].push_back(ea / (p.accel.density / std::sqrt(dt)));
      if (std::abs(dt - 0.004) < 1e-12) {  // the 4 ms steps
        raw4[a].push_back(eg);
        raw4[3 + a].push_back(ea);
      }
    }
  }
  for (int a = 0; a < 6; ++a) {
    // Mean of n N(0,1) values: 5 standard errors.
    EXPECT_NEAR(mean(norm[a]), 0.0, 5.0 / std::sqrt(static_cast<double>(n))) << "axis " << a;
    EXPECT_NEAR(std_dev(norm[a]), 1.0, kStatTol) << "axis " << a;
    const double density = a < 3 ? p.gyro.density : p.accel.density;
    const double sigma4 = density / std::sqrt(0.004);
    EXPECT_NEAR(std_dev(raw4[a]) / sigma4, 1.0, kStatTol) << "axis " << a;
  }
}

// The logged bias is b0 + b(k): with the drift off it is constant over the
// run, and over many runs (seeds) its spread is sigma0.
TEST(ImuCorruptor, TurnOnBiasIsConstantAndHasItsSpread)
{
  ImuErrorParams p = nominal();
  p.gyro.drift_sigma = 0.0;
  p.accel.drift_sigma = 0.0;
  std::vector<double> gb, ab;
  for (std::uint64_t seed = 1; seed <= 2000; ++seed) {
    ImuCorruptor c(p, seed);
    const auto r = run(c, 50);
    for (int a = 0; a < 3; ++a) {
      ASSERT_EQ(r.front().gyro_bias[a], r.back().gyro_bias[a]);
      gb.push_back(r.front().gyro_bias[a]);
      ab.push_back(r.front().accel_bias[a]);
    }
  }
  EXPECT_NEAR(std_dev(gb) / p.gyro.turn_on_sigma, 1.0, kStatTol);
  EXPECT_NEAR(std_dev(ab) / p.accel.turn_on_sigma, 1.0, kStatTol);
}

// With the turn-on bias off the logged bias is the Gauss-Markov part: it
// moves, and over many runs its spread is sigma_ss (stationary from start).
TEST(ImuCorruptor, DriftBiasMovesAndHasItsSpread)
{
  ImuErrorParams p = nominal();
  p.gyro.turn_on_sigma = 0.0;
  p.accel.turn_on_sigma = 0.0;
  std::vector<double> gb, ab;
  for (std::uint64_t seed = 1; seed <= 2000; ++seed) {
    ImuCorruptor c(p, seed);
    const auto r = run(c, 50);
    for (int a = 0; a < 3; ++a) {
      ASSERT_NE(r.front().gyro_bias[a], r.back().gyro_bias[a]);
      gb.push_back(r.back().gyro_bias[a]);
      ab.push_back(r.back().accel_bias[a]);
    }
  }
  EXPECT_NEAR(std_dev(gb) / p.gyro.drift_sigma, 1.0, kStatTol);
  EXPECT_NEAR(std_dev(ab) / p.accel.drift_sigma, 1.0, kStatTol);
}

}  // namespace
