#include "asv_sim/maneuver.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace asv_sim
{

Maneuver::Maneuver(std::vector<ManeuverRow> rows)
: rows_(std::move(rows))
{
  if (rows_.size() < 2) {
    throw std::invalid_argument("maneuver needs at least two rows (a command and the stop row)");
  }
  if (rows_.front().time != 0.0) {
    throw std::invalid_argument("maneuver: the first time must be 0");
  }
  for (std::size_t k = 0; k < rows_.size(); ++k) {
    const auto & r = rows_[k];
    if (!std::isfinite(r.time) || !std::isfinite(r.throttle) || !std::isfinite(r.steer) ||
      !std::isfinite(r.spin))
    {
      throw std::invalid_argument("maneuver: row " + std::to_string(k) + " is not finite");
    }
    if (k > 0 && !(r.time > rows_[k - 1].time)) {
      throw std::invalid_argument(
              "maneuver: times must increase strictly (row " + std::to_string(k) + ")");
    }
  }
  const auto & last = rows_.back();
  if (last.throttle != 0.0 || last.steer != 0.0 || last.spin != 0.0) {
    throw std::invalid_argument("maneuver: the last row marks the end and must be all zero");
  }
}

Maneuver Maneuver::from_columns(
  const std::vector<double> & times, const std::vector<double> & throttle,
  const std::vector<double> & steer, const std::vector<double> & spin)
{
  if (throttle.size() != times.size() || steer.size() != times.size() ||
    spin.size() != times.size())
  {
    throw std::invalid_argument("maneuver: times, throttle, steer and spin differ in length");
  }
  std::vector<ManeuverRow> rows;
  for (std::size_t k = 0; k < times.size(); ++k) {
    rows.push_back({times[k], throttle[k], steer[k], spin[k]});
  }
  return Maneuver(std::move(rows));
}

int Maneuver::active_row(double tau) const
{
  if (tau < 0.0 || tau >= end_time()) {
    return -1;
  }
  // The last k with t_k <= tau; tables are short, a linear search is enough.
  int k = 0;
  while (k + 1 < static_cast<int>(rows_.size()) && rows_[k + 1].time <= tau) {
    ++k;
  }
  return k;
}

ManeuverRow Maneuver::command_at(double tau) const
{
  const int k = active_row(tau);
  if (k < 0) {
    return {tau, 0.0, 0.0, 0.0};
  }
  ManeuverRow c = rows_[k];
  c.time = tau;
  return c;
}

ManeuverPlayer::ManeuverPlayer(
  Maneuver maneuver, double start_delay, const ThrusterLimits & limits)
: maneuver_(std::move(maneuver)), start_delay_(start_delay), limits_(limits)
{
  if (!(start_delay >= 0.0)) {
    throw std::invalid_argument("start_delay must be >= 0");
  }
}

PlayerOutput ManeuverPlayer::step(double t)
{
  // Simulation time 0 means no /clock message yet (design.md section 3).
  if (!has_start_ && t > 0.0) {
    has_start_ = true;
    start_time_ = t + start_delay_;
  }
  const ThrusterCommand zero = mix(0.0, 0.0, 0.0, limits_);
  if (!has_start_ || t < start_time_) {
    return {PlayerState::WAITING, "WAITING", zero};
  }
  const double tau = t - start_time_;
  const int k = maneuver_.active_row(tau);
  if (k < 0) {  // tau >= t_N: the end (tau < 0 is excluded above)
    return {PlayerState::DONE, "DONE", zero};
  }
  const ManeuverRow c = maneuver_.command_at(tau);
  return {PlayerState::RUNNING,
    "RUNNING " + std::to_string(k + 1) + "/" + std::to_string(maneuver_.command_rows()),
    mix(c.throttle, c.steer, c.spin, limits_)};
}

}  // namespace asv_sim
