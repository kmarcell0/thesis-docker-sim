// GNSS corruptor (M2.6b position, M2.6c velocity and heading); see
// gnss_corruptor.hpp for the equations.
#include "asv_core/gnss_corruptor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "asv_core/angles.hpp"

namespace asv_core
{

GnssMode gnss_mode_from_name(const std::string & name)
{
  if (name == "outage") {return GnssMode::outage;}
  if (name == "standalone") {return GnssMode::standalone;}
  if (name == "rtk_float") {return GnssMode::rtk_float;}
  if (name == "rtk_fixed") {return GnssMode::rtk_fixed;}
  throw std::invalid_argument(
          "unknown GNSS mode '" + name + "' (outage, standalone, rtk_float, rtk_fixed)");
}

std::string gnss_mode_name(GnssMode mode)
{
  switch (mode) {
    case GnssMode::outage: return "outage";
    case GnssMode::standalone: return "standalone";
    case GnssMode::rtk_float: return "rtk_float";
    case GnssMode::rtk_fixed: return "rtk_fixed";
  }
  throw std::invalid_argument("invalid GnssMode");
}

GnssSchedule::GnssSchedule(std::vector<GnssScheduleEntry> entries)
: entries_(std::move(entries))
{
  if (entries_.empty()) {
    throw std::invalid_argument("GNSS schedule is empty");
  }
  if (entries_.front().from != 0.0) {
    throw std::invalid_argument("GNSS schedule must start at from = 0");
  }
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    if (!std::isfinite(entries_[i].from) ||
      (i > 0 && !(entries_[i].from > entries_[i - 1].from)))
    {
      throw std::invalid_argument(
              "GNSS schedule: entry " + std::to_string(i) +
              " must start later than the one before");
    }
  }
}

GnssMode GnssSchedule::mode_at(double t) const
{
  GnssMode mode = entries_.front().mode;
  for (const auto & e : entries_) {
    if (e.from > t) {break;}
    mode = e.mode;
  }
  return mode;
}

GnssCorruptor::Axis::Axis(
  double white_sigma_, double slow_sigma, double drift_time, std::uint64_t seed,
  const std::string & mode, char axis)
: white_sigma(white_sigma_),
  white(seed, "gnss." + mode + ".white_" + axis),
  slow(slow_sigma, drift_time, seed, "gnss." + mode + ".slow_" + axis)
{
}

GnssCorruptor::ModeAxes GnssCorruptor::make_axes(
  const GnssModeErrorParams & p, std::uint64_t seed, const std::string & mode)
{
  return ModeAxes{
    Axis(p.white_h, p.slow_h, p.drift_time, seed, mode, 'e'),
    Axis(p.white_h, p.slow_h, p.drift_time, seed, mode, 'n'),
    Axis(p.white_v, p.slow_v, p.drift_time, seed, mode, 'u')};
}

GnssCorruptor::GnssCorruptor(
  const GnssErrorParams & params, GnssSchedule schedule, std::uint64_t global_seed)
: modes_{make_axes(params.standalone, global_seed, "standalone"),
    make_axes(params.rtk_float, global_seed, "rtk_float"),
    make_axes(params.rtk_fixed, global_seed, "rtk_fixed")},
  velocity_white_(params.velocity_white),
  velocity_streams_{RandomStream(global_seed, "gnss.velocity_e"),
    RandomStream(global_seed, "gnss.velocity_n"),
    RandomStream(global_seed, "gnss.velocity_u")},
  heading_white_(params.heading_white),
  heading_stream_(global_seed, "gnss.heading"),
  schedule_(std::move(schedule))
{
  for (const auto * p : {&params.standalone, &params.rtk_float, &params.rtk_fixed}) {
    if (!(p->white_h >= 0.0 && p->white_v >= 0.0)) {
      throw std::invalid_argument("GNSS white sigma must be >= 0");
    }
  }
  if (!(velocity_white_ >= 0.0) || (heading_white_ && !(*heading_white_ >= 0.0))) {
    throw std::invalid_argument("GNSS velocity and heading sigma must be >= 0");
  }
}

GnssCorruptor::ModeAxes & GnssCorruptor::axes_of(GnssMode mode)
{
  // standalone = 1, rtk_float = 2, rtk_fixed = 3 -> index 0, 1, 2
  return modes_[static_cast<std::size_t>(mode) - 1];
}

std::optional<GnssEpoch> GnssCorruptor::process(std::int64_t stamp_ns)
{
  if (last_stamp_ns_ && stamp_ns <= *last_stamp_ns_) {
    return std::nullopt;  // old or repeated stamp
  }
  if (!first_stamp_ns_) {
    first_stamp_ns_ = stamp_ns;
  }
  last_stamp_ns_ = stamp_ns;

  GnssEpoch out;
  // Integer nanoseconds, so t and dt have no rounding error at large stamps.
  const double t = static_cast<double>(stamp_ns - *first_stamp_ns_) * 1e-9;
  out.mode = schedule_.mode_at(t);
  if (out.mode == GnssMode::outage) {
    return out;  // no fix; the slow errors wait for the next fix
  }

  ModeAxes & axes = axes_of(out.mode);
  const bool same_mode = last_fix_mode_ && *last_fix_mode_ == out.mode;
  // Same mode: step s over the time since the last fix (also across an
  // outage, Q3 a). New mode (also the first fix): fresh draw.
  const double dt = static_cast<double>(stamp_ns - last_fix_stamp_ns_) * 1e-9;
  Eigen::Vector3d error;
  for (int j = 0; j < 3; ++j) {
    Axis & a = axes[static_cast<std::size_t>(j)];
    out.slow_error[j] = same_mode ? a.slow.step(dt) : a.slow.redraw();
    error[j] = out.slow_error[j] + a.white_sigma * a.white.normal();  // e = s + n
    out.sigma[j] = std::hypot(a.white_sigma, a.slow.sigma_ss());
  }
  out.error = error;

  // Velocity and heading: white, the same in every mode (Q3 a, Q4 a).
  Eigen::Vector3d v_noise;
  for (std::size_t j = 0; j < 3; ++j) {
    v_noise[static_cast<int>(j)] = velocity_white_ * velocity_streams_[j].normal();
  }
  out.velocity_noise = v_noise;
  if (heading_white_) {
    out.heading_noise = *heading_white_ * heading_stream_.normal();
  }

  last_fix_mode_ = out.mode;
  last_fix_stamp_ns_ = stamp_ns;
  return out;
}

double azimuth_from_yaw(double psi)
{
  // wrap_angle gives (-pi, pi]; shift the negative half up by 2 pi. A value
  // that rounds to exactly 2 pi becomes 0, so the result is in [0, 2 pi).
  double alpha = wrap_angle(M_PI / 2.0 - psi);
  if (alpha < 0.0) {
    alpha += 2.0 * M_PI;
  }
  return alpha >= 2.0 * M_PI ? 0.0 : alpha;
}

Eigen::Vector3d central_difference(
  const Eigen::Vector3d & p_prev, std::int64_t t_prev_ns,
  const Eigen::Vector3d & p_next, std::int64_t t_next_ns)
{
  if (!(t_next_ns > t_prev_ns)) {
    throw std::invalid_argument("central_difference: t_next must be later than t_prev");
  }
  return (p_next - p_prev) / (static_cast<double>(t_next_ns - t_prev_ns) * 1e-9);
}

YawHistory::YawHistory(std::int64_t keep_ns)
: keep_ns_(keep_ns)
{
  if (keep_ns <= 0) {
    throw std::invalid_argument("YawHistory: keep_ns must be > 0");
  }
}

void YawHistory::add(std::int64_t stamp_ns, double psi)
{
  if (!samples_.empty() && stamp_ns <= samples_.back().first) {
    return;
  }
  samples_.emplace_back(stamp_ns, psi);
  while (samples_.front().first < stamp_ns - keep_ns_) {
    samples_.pop_front();
  }
}

std::optional<double> YawHistory::at(std::int64_t stamp_ns) const
{
  if (samples_.empty() || stamp_ns < samples_.front().first ||
    stamp_ns > samples_.back().first)
  {
    return std::nullopt;
  }
  // First sample not earlier than the stamp; the one before brackets it.
  std::size_t i = 0;
  while (samples_[i].first < stamp_ns) {++i;}
  if (samples_[i].first == stamp_ns) {
    return samples_[i].second;
  }
  const auto & a = samples_[i - 1];
  const auto & b = samples_[i];
  const double f = static_cast<double>(stamp_ns - a.first) /
    static_cast<double>(b.first - a.first);
  return wrap_angle(a.second + f * wrap_angle(b.second - a.second));
}

std::optional<std::int64_t> YawHistory::newest() const
{
  if (samples_.empty()) {return std::nullopt;}
  return samples_.back().first;
}

std::optional<std::int64_t> YawHistory::oldest() const
{
  if (samples_.empty()) {return std::nullopt;}
  return samples_.front().first;
}

}  // namespace asv_core
