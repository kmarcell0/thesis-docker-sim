// GNSS position corruptor (M2.6b); see gnss_corruptor.hpp for the equations.
#include "asv_core/gnss_corruptor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

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
  schedule_(std::move(schedule))
{
  for (const auto * p : {&params.standalone, &params.rtk_float, &params.rtk_fixed}) {
    if (!(p->white_h >= 0.0 && p->white_v >= 0.0)) {
      throw std::invalid_argument("GNSS white sigma must be >= 0");
    }
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
  last_fix_mode_ = out.mode;
  last_fix_stamp_ns_ = stamp_ns;
  return out;
}

}  // namespace asv_core
