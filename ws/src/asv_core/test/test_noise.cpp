// Unit tests for the noise library (M2.5): design.md 4.5.
// The three statistics tests use long records with fixed seeds. Their 5 %
// tolerance was set from 10 seeds (M2.5 report): worst error 2.08 % with
// 40 h records for tests 2 and 3, 0.70 % with 10 h for test 1.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "allan.hpp"
#include "asv_core/noise.hpp"

namespace
{

using asv_core::GaussMarkov;
using asv_core::RandomStream;
using asv_core::TurnOnBias;
using asv_core::WhiteNoise;
using asv_core::stream_seed;
using asv_core_test::allan_deviation;
using asv_core_test::autocorrelation;
using asv_core_test::gauss_markov_allan_variance;
using asv_core_test::mean;
using asv_core_test::std_dev;

// Nominal IMU values [REPO asv.yaml sensor_specs]: noise densities [DS],
// biases and correlation time [EST].
constexpr double kGyroDensity = 6.632e-5;   // rad/s/sqrt(Hz)
constexpr double kAccelDensity = 6.865e-4;  // m/s^2/sqrt(Hz)
constexpr double kGyroBias = 0.0017;        // rad/s, turn-on
constexpr double kAccelBias = 0.02;         // m/s^2, turn-on
constexpr double kGyroDrift = 0.0002;       // rad/s, Gauss-Markov sigma_ss
constexpr double kAccelDrift = 0.005;       // m/s^2, Gauss-Markov sigma_ss
constexpr double kDriftTime = 300.0;        // s, Gauss-Markov T_c

constexpr double kDt = 0.005;    // s, 200 Hz IMU (even steps for Allan)
constexpr double kStatTol = 0.05;  // relative, design.md 4.5

std::size_t samples(double hours) {return static_cast<std::size_t>(hours * 3600.0 / kDt);}
std::size_t cluster(double tau) {return static_cast<std::size_t>(std::lround(tau / kDt));}

// The IMU's real step pattern: samples on 4 ms physics steps, 8, 4, 4, 4 ms
// (design.md 3, rule 4). Four samples per 20 ms, so 200 samples = 1 s.
double uneven_dt(std::size_t i)
{
  return (i % 4 == 0) ? 0.008 : 0.004;
}

// ---- random streams -------------------------------------------------------

TEST(RandomStream, SeedDependsOnGlobalSeedAndName)
{
  EXPECT_EQ(stream_seed(42, "imu.gyro_z.white"), stream_seed(42, "imu.gyro_z.white"));
  EXPECT_NE(stream_seed(42, "imu.gyro_z.white"), stream_seed(43, "imu.gyro_z.white"));
  EXPECT_NE(stream_seed(42, "imu.gyro_z.white"), stream_seed(42, "imu.gyro_y.white"));
}

TEST(RandomStream, SameSeedSameNumbersOtherNameOtherNumbers)
{
  RandomStream a(7, "gnss.east"), b(7, "gnss.east"), c(7, "gnss.north");
  int equal_to_c = 0;
  for (int i = 0; i < 1000; ++i) {
    const double va = a.normal();
    ASSERT_EQ(va, b.normal());
    if (va == c.normal()) {++equal_to_c;}
  }
  EXPECT_EQ(equal_to_c, 0);
}

// Pinned values: if the seeding or the normal draw ever changes, every
// recorded run's noise changes, and this test says so.
TEST(RandomStream, PinnedValues)
{
  EXPECT_EQ(stream_seed(42, "imu.gyro_z.white"), 2501700126879760238ULL);
  RandomStream s(42, "imu.gyro_z.white");
  EXPECT_NEAR(s.normal(), -0.55795909552623568, 1e-12);
  EXPECT_NEAR(s.normal(), -0.43332477262422398, 1e-12);
  EXPECT_NEAR(s.normal(), 0.55176351523534706, 1e-12);
}

TEST(RandomStream, StandardNormalMoments)
{
  RandomStream s(1, "moments");
  std::vector<double> w(1000000);
  for (auto & v : w) {v = s.normal();}
  // Standard errors: mean 0.001, std 0.0007; 5 sigma bounds.
  EXPECT_NEAR(mean(w), 0.0, 0.005);
  EXPECT_NEAR(std_dev(w), 1.0, 0.0035);
  // Tail check: P(|w| > 2) = 4.55 %.
  std::size_t tail = 0;
  for (const double v : w) {if (std::fabs(v) > 2.0) {++tail;}}
  EXPECT_NEAR(static_cast<double>(tail) / w.size(), 0.0455, 0.001);
}

// ---- white noise ----------------------------------------------------------

TEST(WhiteNoise, SigmaFromDensity)
{
  WhiteNoise n(kGyroDensity, 1, "gyro");
  // 200 Hz: 6.632e-5 / sqrt(0.005) = 9.379e-4 rad/s (design.md 4.1)
  EXPECT_NEAR(n.sigma(0.005), kGyroDensity / std::sqrt(0.005), 1e-18);
  EXPECT_NEAR(n.sigma(0.005), 9.379e-4, 1e-7);
  EXPECT_NEAR(n.sigma(0.004), kGyroDensity / std::sqrt(0.004), 1e-18);
}

TEST(WhiteNoise, RejectsBadInput)
{
  WhiteNoise n(kGyroDensity, 1, "gyro");
  EXPECT_THROW(n.sample(0.0), std::invalid_argument);
  EXPECT_THROW(n.sample(-0.004), std::invalid_argument);
  EXPECT_THROW(WhiteNoise(-1.0, 1, "gyro"), std::invalid_argument);
}

// With sigma = eta / sqrt(dt), the integral of the noise over a window T has
// variance eta^2 T whatever the step pattern is. Uneven 8/4/4/4 ms steps,
// windows of 1 s (200 samples), 20000 windows (std error of the variance 1 %).
TEST(WhiteNoise, IntegralVarianceWithUnevenSteps)
{
  WhiteNoise n(kGyroDensity, 3, "gyro");
  std::vector<double> integrals(20000);
  std::size_t i = 0;
  for (auto & integral : integrals) {
    integral = 0.0;
    for (int k = 0; k < 200; ++k, ++i) {
      const double dt = uneven_dt(i);
      integral += n.sample(dt) * dt;
    }
  }
  const double sd = std_dev(integrals);
  EXPECT_NEAR(sd / (kGyroDensity * std::sqrt(1.0)), 1.0, kStatTol);
}

// ---- turn-on bias ---------------------------------------------------------

TEST(TurnOnBias, ConstantPerRunSpreadOverRuns)
{
  TurnOnBias b(kGyroBias, 5, "gyro_z.bias");
  EXPECT_EQ(b.value(), b.value());
  EXPECT_EQ(b.value(), TurnOnBias(kGyroBias, 5, "gyro_z.bias").value());
  // 20000 runs (global seeds): spread sigma0, std error 0.5 %.
  std::vector<double> draws(20000);
  for (std::size_t s = 0; s < draws.size(); ++s) {
    draws[s] = TurnOnBias(kGyroBias, s, "gyro_z.bias").value();
  }
  EXPECT_NEAR(std_dev(draws) / kGyroBias, 1.0, 0.025);
  EXPECT_THROW(TurnOnBias(-1.0, 1, "x"), std::invalid_argument);
}

// ---- Gauss-Markov ---------------------------------------------------------

TEST(GaussMarkov, RejectsBadInput)
{
  GaussMarkov g(kGyroDrift, kDriftTime, 1, "gyro");
  EXPECT_THROW(g.step(0.0), std::invalid_argument);
  EXPECT_THROW(g.step(-1.0), std::invalid_argument);
  EXPECT_THROW(GaussMarkov(kGyroDrift, 0.0, 1, "gyro"), std::invalid_argument);
  EXPECT_THROW(GaussMarkov(-1.0, kDriftTime, 1, "gyro"), std::invalid_argument);
}

TEST(GaussMarkov, ZeroSizeStaysZero)
{
  GaussMarkov g(0.0, kDriftTime, 1, "off");
  EXPECT_EQ(g.value(), 0.0);
  for (int i = 0; i < 100; ++i) {EXPECT_EQ(g.step(kDt), 0.0);}
}

// Stationary start (decision B a): over many runs the very first value already
// has spread sigma_ss.
TEST(GaussMarkov, StartsStationary)
{
  std::vector<double> starts(20000);
  for (std::size_t s = 0; s < starts.size(); ++s) {
    starts[s] = GaussMarkov(kGyroDrift, kDriftTime, s, "gyro").value();
  }
  EXPECT_NEAR(std_dev(starts) / kGyroDrift, 1.0, 0.025);
}

// The exact step keeps the variance and the correlation e^(-t/T_c) with the
// IMU's uneven steps. T_c = 5 s, 40 h; lag 200 samples = exactly 1 s.
TEST(GaussMarkov, UnevenStepsKeepVarianceAndCorrelation)
{
  const double tc = 5.0;
  GaussMarkov g(kGyroDrift, tc, 11, "gyro");
  std::vector<double> b(samples(40.0));
  for (std::size_t i = 0; i < b.size(); ++i) {b[i] = g.step(uneven_dt(i));}
  EXPECT_NEAR(std_dev(b) / kGyroDrift, 1.0, kStatTol);
  EXPECT_NEAR(autocorrelation(b, 200) / std::exp(-1.0 / tc), 1.0, kStatTol);
}

// ---- Allan deviation helper ------------------------------------------------

// A rate that grows linearly, y = a t: neighbouring tau-averages differ by
// exactly a tau, so sigma_A = a tau / sqrt(2). Checks the checker.
TEST(AllanHelper, RampGivesATauOverSqrt2)
{
  const double a = 0.01;
  std::vector<double> y(10000);
  for (std::size_t i = 0; i < y.size(); ++i) {y[i] = a * i * kDt;}
  for (const std::size_t m : {1u, 10u, 100u, 1000u}) {
    const double tau = m * kDt;
    EXPECT_NEAR(allan_deviation(y, kDt, m), a * tau / std::sqrt(2.0), 1e-9 * a * tau);
  }
  const std::vector<double> constant(1000, 3.0);
  EXPECT_NEAR(allan_deviation(constant, kDt, 10), 0.0, 1e-9);
}

// ---- design.md 4.5, the three noise tests ---------------------------------

// Test 1: white noise alone; Allan deviation = eta / sqrt(tau). 10 h.
TEST(NoiseModel, Test1WhiteAloneAllan)
{
  WhiteNoise n(kGyroDensity, 1, "test1");
  std::vector<double> y(samples(10.0));
  for (auto & v : y) {v = n.sample(kDt);}
  for (const double tau : {0.1, 1.0, 10.0}) {
    const double expected = kGyroDensity / std::sqrt(tau);
    EXPECT_NEAR(allan_deviation(y, kDt, cluster(tau)) / expected, 1.0, kStatTol)
      << "tau = " << tau << " s";
  }
}

// Test 2: Gauss-Markov alone with a short test T_c = 5 s, 40 h: spread
// sigma_ss, lag correlation e^(-k dt/T_c), Allan deviation on the analytic
// curve at 10 values of tau from 0.5 T_c to 4 T_c (log spaced).
TEST(NoiseModel, Test2GaussMarkovAlone)
{
  const double tc = 5.0;
  GaussMarkov g(kGyroDrift, tc, 1, "test2");
  std::vector<double> y(samples(40.0));
  for (auto & v : y) {v = g.step(kDt);}

  EXPECT_NEAR(std_dev(y) / kGyroDrift, 1.0, kStatTol);
  for (const double lag : {1.0, 5.0}) {
    EXPECT_NEAR(autocorrelation(y, cluster(lag)) / std::exp(-lag / tc), 1.0, kStatTol)
      << "lag = " << lag << " s";
  }
  for (int i = 0; i < 10; ++i) {
    const std::size_t m = cluster(0.5 * tc * std::pow(8.0, i / 9.0));
    const double tau = m * kDt;
    const double expected = std::sqrt(gauss_markov_allan_variance(kGyroDrift, tc, tau));
    EXPECT_NEAR(allan_deviation(y, kDt, m) / expected, 1.0, kStatTol)
      << "tau = " << tau << " s";
  }
}

// Test 3: everything together with nominal values, 40 h: the Allan deviation
// is sqrt(eta^2/tau + sigma_A^2(tau)) at tau = 1, 10, 30 s. The turn-on bias is
// constant, so it does not change the Allan deviation; it is added anyway so
// the test uses the full model measured = true + b0 + b_k + n_k (true = 0).
void check_full_model(double density, double bias, double drift, const std::string & name)
{
  WhiteNoise n(density, 1, name + ".white");
  TurnOnBias b0(bias, 1, name + ".bias");
  GaussMarkov bk(drift, kDriftTime, 1, name + ".drift");
  std::vector<double> y(samples(40.0));
  for (auto & v : y) {v = b0.value() + bk.step(kDt) + n.sample(kDt);}
  for (const double tau : {1.0, 10.0, 30.0}) {
    const double expected = std::sqrt(
      density * density / tau + gauss_markov_allan_variance(drift, kDriftTime, tau));
    EXPECT_NEAR(allan_deviation(y, kDt, cluster(tau)) / expected, 1.0, kStatTol)
      << name << ", tau = " << tau << " s";
  }
}

TEST(NoiseModel, Test3FullModelGyro)
{
  check_full_model(kGyroDensity, kGyroBias, kGyroDrift, "gyro_z");
}

TEST(NoiseModel, Test3FullModelAccel)
{
  check_full_model(kAccelDensity, kAccelBias, kAccelDrift, "accel_x");
}

// Without the bias the 10 s and 30 s values must miss by far more than the
// tolerance (design.md 4.5: +72 % gyro, +254 % accel at 10 s), so a model
// that forgets the bias cannot pass test 3.
TEST(NoiseModel, Test3WouldCatchMissingBias)
{
  for (const auto & [density, drift] :
    {std::pair{kGyroDensity, kGyroDrift}, std::pair{kAccelDensity, kAccelDrift}})
  {
    const double tau = 10.0;
    const double white_only = density / std::sqrt(tau);
    const double total = std::sqrt(
      white_only * white_only + gauss_markov_allan_variance(drift, kDriftTime, tau));
    EXPECT_GT(total / white_only - 1.0, 10 * kStatTol);
  }
}

}  // namespace
