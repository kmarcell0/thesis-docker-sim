// Noise library (M2.5); see noise.hpp for the equations.
#include "asv_core/noise.hpp"

#include <cmath>
#include <stdexcept>

namespace asv_core
{

namespace
{

// FNV-1a, 64-bit: turns the source name into a number.
std::uint64_t fnv1a(const std::string & text)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char c : text) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
  return hash;
}

// SplitMix64 finaliser: mixes the bits, so seeds that differ in one bit give
// unrelated engine states.
std::uint64_t splitmix64(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

void require_non_negative(double value, const char * what)
{
  if (!(value >= 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(what) + " must be finite and >= 0");
  }
}

void require_positive(double value, const char * what)
{
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(what) + " must be finite and > 0");
  }
}

}  // namespace

std::uint64_t stream_seed(std::uint64_t global_seed, const std::string & source)
{
  return splitmix64(global_seed ^ fnv1a(source));
}

RandomStream::RandomStream(std::uint64_t global_seed, const std::string & source)
: engine_(stream_seed(global_seed, source))
{
}

double RandomStream::uniform()
{
  // 53 bits fill a double's mantissa exactly: u in [0, 1) with step 2^-53.
  return static_cast<double>(engine_() >> 11) * 0x1.0p-53;
}

double RandomStream::uniform_pm1()
{
  return 2.0 * uniform() - 1.0;
}

double RandomStream::normal()
{
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  // Marsaglia polar method: a random point (u, v) inside the unit circle,
  // s = u^2 + v^2, gives two independent N(0, 1) draws u*f and v*f with
  // f = sqrt(-2 ln(s) / s).
  double u, v, s;
  do {
    u = uniform_pm1();
    v = uniform_pm1();
    s = u * u + v * v;
  } while (s >= 1.0 || s == 0.0);
  const double f = std::sqrt(-2.0 * std::log(s) / s);
  spare_ = v * f;
  has_spare_ = true;
  return u * f;
}

WhiteNoise::WhiteNoise(
  double density, std::uint64_t global_seed, const std::string & source)
: density_(density), stream_(global_seed, source)
{
  require_non_negative(density, "noise density");
}

double WhiteNoise::sigma(double dt) const
{
  require_positive(dt, "dt");
  return density_ / std::sqrt(dt);  // sigma_w = eta / sqrt(dt)
}

double WhiteNoise::sample(double dt)
{
  const double sigma_w = sigma(dt);
  return sigma_w * stream_.normal();  // n_k = sigma_w * w_k
}

TurnOnBias::TurnOnBias(
  double sigma, std::uint64_t global_seed, const std::string & source)
{
  require_non_negative(sigma, "turn-on bias sigma");
  RandomStream stream(global_seed, source);
  value_ = sigma * stream.normal();  // b0 ~ N(0, sigma0^2)
}

GaussMarkov::GaussMarkov(
  double sigma_ss, double corr_time, std::uint64_t global_seed,
  const std::string & source)
: sigma_ss_(sigma_ss), corr_time_(corr_time), stream_(global_seed, source)
{
  require_non_negative(sigma_ss, "Gauss-Markov sigma_ss");
  require_positive(corr_time, "Gauss-Markov correlation time");
  value_ = sigma_ss_ * stream_.normal();  // stationary start, N(0, sigma_ss^2)
}

double GaussMarkov::step(double dt)
{
  require_positive(dt, "dt");
  const double phi = std::exp(-dt / corr_time_);
  // sqrt(1 - phi^2) via expm1 stays accurate when dt << T_c (phi close to 1).
  const double push = sigma_ss_ * std::sqrt(-std::expm1(-2.0 * dt / corr_time_));
  value_ = phi * value_ + push * stream_.normal();
  return value_;
}

double GaussMarkov::redraw()
{
  value_ = sigma_ss_ * stream_.normal();  // N(0, sigma_ss^2), as at the start
  return value_;
}

}  // namespace asv_core
