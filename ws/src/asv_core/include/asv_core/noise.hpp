// Noise library (M2.5): the sensor error models of the corruptor, ROS-free.
// Thesis: "Noise Models" (design.md sections 4.1 and 4.5).
//
// A corrupted reading is   measured = true + b0 + b_k + n_k   with
//   n_k  white noise        (WhiteNoise),
//   b0   turn-on bias       (TurnOnBias, one draw per run),
//   b_k  wandering bias     (GaussMarkov, first-order Gauss-Markov process).
// Each of them owns its own random stream, seeded from (global seed, source
// name), so changing or switching off one source never changes the numbers of
// another ("common random numbers", glossary). Time steps dt come from the
// message stamps, never from the nominal rate (design.md section 3).
#pragma once

#include <cstdint>
#include <random>
#include <string>

namespace asv_core
{

// 64-bit seed of the stream called `source` in a run with `global_seed`:
// SplitMix64(global_seed XOR FNV-1a(source)). Both hashes are fixed
// algorithms, so the seed is the same on every compiler and machine.
std::uint64_t stream_seed(std::uint64_t global_seed, const std::string & source);

// One seeded random stream. The engine std::mt19937_64 is fully specified by
// the C++ standard; std::normal_distribution is not (its output differs
// between standard libraries), so the normal draw is done here (Marsaglia's
// polar method, design.md 4.5).
class RandomStream
{
public:
  RandomStream(std::uint64_t global_seed, const std::string & source);

  // Standard normal draw w ~ N(0, 1).
  double normal();

  // Uniform draw u in [0, 1), from the top 53 bits of one engine output
  // (M3.1: misses, false alarms and colour mix-ups of the fake detector).
  double uniform();

private:
  // Uniform draw in [-1, 1): 2 u - 1.
  double uniform_pm1();

  std::mt19937_64 engine_;
  bool has_spare_ = false;  // the polar method makes two draws at a time
  double spare_ = 0.0;
};

// White noise from a noise density (design.md 4.1, decision A a of M2.5):
//   n_k = sigma_w * w_k,   sigma_w = eta / sqrt(dt)
// eta: noise density (unit of the signal per sqrt(Hz), e.g. rad/s/sqrt(Hz)),
// dt: time since the previous sample (s). With this rule the Allan deviation
// of the noise is eta / sqrt(tau), and the integral of the noise over a time T
// has variance eta^2 T whatever the sample times are.
class WhiteNoise
{
public:
  WhiteNoise(double density, std::uint64_t global_seed, const std::string & source);

  // Per-sample standard deviation sigma_w for a step dt (s). Throws
  // std::invalid_argument if dt is not > 0.
  double sigma(double dt) const;

  // One noise value n_k for a sample taken dt seconds after the previous one.
  double sample(double dt);

private:
  double density_;
  RandomStream stream_;
};

// Turn-on bias (design.md 4.1): b0 ~ N(0, sigma0^2), drawn once when the
// object is made, then constant for the whole run.
class TurnOnBias
{
public:
  TurnOnBias(double sigma, std::uint64_t global_seed, const std::string & source);

  double value() const {return value_;}

private:
  double value_;
};

// First-order Gauss-Markov process, exact discrete step (design.md 4.1):
//   phi_k   = exp(-dt_k / T_c)
//   b_{k+1} = phi_k * b_k + sigma_ss * sqrt(1 - phi_k^2) * w_k
// sigma_ss: steady-state standard deviation, T_c: correlation time (s).
// The push size sqrt(1 - phi^2) keeps the variance at sigma_ss^2 for any dt,
// so uneven IMU steps (8, 4, 4, 4 ms) need no special care. The start value
// is drawn from N(0, sigma_ss^2) (decision B a of M2.5), so the process is
// stationary from the first sample.
class GaussMarkov
{
public:
  GaussMarkov(
    double sigma_ss, double corr_time, std::uint64_t global_seed,
    const std::string & source);

  // Current value b_k.
  double value() const {return value_;}

  double sigma_ss() const {return sigma_ss_;}

  // Advances the process by dt (s, > 0, else std::invalid_argument) and
  // returns the new value b_{k+1}.
  double step(double dt);

  // Replaces the current value by a fresh draw from N(0, sigma_ss^2), from the
  // same stream: the process forgets its past. The GNSS corruptor uses it at
  // a receiver mode switch (M2.6b, design.md 4.2).
  double redraw();

private:
  double sigma_ss_;
  double corr_time_;
  RandomStream stream_;
  double value_;
};

}  // namespace asv_core
