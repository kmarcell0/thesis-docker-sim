// Statistics helpers for the noise tests (M2.5), test-only.
// Overlapping Allan deviation, the analytic Allan variance of a first-order
// Gauss-Markov process, sample standard deviation and autocorrelation.
#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace asv_core_test
{

// Overlapping Allan deviation of rate samples y (e.g. rad/s) taken every
// dt0 seconds, at tau = m * dt0. With the integral theta_k = dt0 * sum_{i<k} y_i
// (k = 0..N):
//   sigma_A^2(tau) = sum_k (theta_{k+2m} - 2 theta_{k+m} + theta_k)^2
//                    / (2 tau^2 (N + 1 - 2m))
// i.e. half the mean squared difference of two neighbouring tau-averages,
// using every start index k (the "overlapping" estimator).
inline double allan_deviation(const std::vector<double> & y, double dt0, std::size_t m)
{
  const std::size_t n = y.size();
  if (m == 0 || 2 * m > n) {
    throw std::invalid_argument("allan_deviation: need 1 <= m <= N/2");
  }
  std::vector<double> theta(n + 1, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    theta[i + 1] = theta[i] + y[i] * dt0;
  }
  const double tau = static_cast<double>(m) * dt0;
  long double sum = 0.0L;
  const std::size_t count = n + 1 - 2 * m;
  for (std::size_t k = 0; k < count; ++k) {
    const double d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k];
    sum += static_cast<long double>(d) * d;
  }
  return std::sqrt(static_cast<double>(sum / count) / (2.0 * tau * tau));
}

// Analytic Allan variance of a Gauss-Markov process (design.md 4.5):
//   sigma_A^2(tau) = (sigma_ss^2 T_c^2 / tau^2)
//                    * [2 tau/T_c - 3 + 4 e^(-tau/T_c) - e^(-2 tau/T_c)]
inline double gauss_markov_allan_variance(double sigma_ss, double corr_time, double tau)
{
  const double x = tau / corr_time;
  return sigma_ss * sigma_ss / (x * x) *
         (2.0 * x - 3.0 + 4.0 * std::exp(-x) - std::exp(-2.0 * x));
}

inline double mean(const std::vector<double> & y)
{
  long double s = 0.0L;
  for (const double v : y) {s += v;}
  return static_cast<double>(s / y.size());
}

inline double std_dev(const std::vector<double> & y)
{
  const double mu = mean(y);
  long double s = 0.0L;
  for (const double v : y) {s += static_cast<long double>(v - mu) * (v - mu);}
  return std::sqrt(static_cast<double>(s / (y.size() - 1)));
}

// Sample autocorrelation at lag k: covariance of (y_i, y_{i+k}) over the
// variance.
inline double autocorrelation(const std::vector<double> & y, std::size_t k)
{
  const double mu = mean(y);
  long double num = 0.0L, den = 0.0L;
  for (std::size_t i = 0; i < y.size(); ++i) {
    const double d = y[i] - mu;
    den += static_cast<long double>(d) * d;
    if (i + k < y.size()) {num += static_cast<long double>(d) * (y[i + k] - mu);}
  }
  return static_cast<double>(num / den);
}

}  // namespace asv_core_test
