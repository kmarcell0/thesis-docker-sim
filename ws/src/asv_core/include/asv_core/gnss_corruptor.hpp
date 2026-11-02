// GNSS position corruptor (M2.6b): the error of one GNSS fix, ROS-free.
// Thesis: "Noise Models", "Vehicle and Sensor Models" (design.md section 4.2).
// Code -> glossary symbols: e = epsilon_G, s = beta_G, white sigma = sigma_w.
//
// The receiver is in one mode m at a time (RTK fixed, RTK float, standalone,
// or a sky-view outage with no fix); the mode over time comes from a
// schedule. For every axis j in {E, N, U} of a fix in mode m:
//   e_j(k) = s_m,j(k) + n_m,j(k)                                  (ENU, m)
//   n_m,j(k) ~ N(0, sigma_w,m,j^2)   white, one draw per epoch (the GNSS
//                                    spec is per epoch, not a density)
//   s_m,j(k) Gauss-Markov (sigma_ss,m,j, T_c,m), stepped by the time between
//                                    the stamps of consecutive fixes
// Mode switch: the new mode's slow error s gets a fresh draw from
// N(0, sigma_ss^2) (the jump seen when RTK comes back). Same mode again after
// an outage: s is stepped over the whole gap, so a short gap keeps nearly the
// same offset and a gap much longer than T_c gives a new one (decision Q3 a,
// 2026-11-02). The receiver reports the total per-axis sigma
// sqrt(sigma_w^2 + sigma_ss^2) (Q2 a, 2026-11-02).
// Each of 3 modes x 3 axes x 2 parts = 18 sources has its own random stream,
// named e.g. "gnss.rtk_fixed.slow_e" (noise.hpp).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "asv_core/noise.hpp"

namespace asv_core
{

// Receiver mode. The numbers are those of asv_msgs/GnssStatus MODE_*.
enum class GnssMode : std::uint8_t
{
  outage = 0,      // sky-view outage: no fix
  standalone = 1,  // no RTK corrections, metre level
  rtk_float = 2,   // corrections used, ambiguities not fixed, decimetre level
  rtk_fixed = 3,   // ambiguities fixed, centimetre level
};

// "outage", "standalone", "rtk_float", "rtk_fixed" <-> GnssMode. An unknown
// name throws std::invalid_argument.
GnssMode gnss_mode_from_name(const std::string & name);
std::string gnss_mode_name(GnssMode mode);

// Error model of one mode, per-axis 1-sigma values in m. h = east and north
// (each), v = up.
struct GnssModeErrorParams
{
  double white_h = 0.0;     // sigma_w, horizontal
  double slow_h = 0.0;      // sigma_ss, horizontal
  double white_v = 0.0;     // sigma_w, vertical
  double slow_v = 0.0;      // sigma_ss, vertical
  double drift_time = 1.0;  // T_c of the slow error (s), > 0
};

struct GnssErrorParams
{
  GnssModeErrorParams standalone;
  GnssModeErrorParams rtk_float;
  GnssModeErrorParams rtk_fixed;
};

// The mode over time: entry i holds from `from` (s after the first epoch)
// until the next entry's `from`; the last one to the end of the run.
struct GnssScheduleEntry
{
  double from;    // s, >= 0
  GnssMode mode;
};

class GnssSchedule
{
public:
  // Throws std::invalid_argument unless the list is not empty, the first
  // entry starts at 0 and every `from` is finite and later than the one
  // before.
  explicit GnssSchedule(std::vector<GnssScheduleEntry> entries);

  // Mode at t seconds after the first epoch (t < 0 counts as 0).
  GnssMode mode_at(double t) const;

  const std::vector<GnssScheduleEntry> & entries() const {return entries_;}

private:
  std::vector<GnssScheduleEntry> entries_;
};

// The outcome of one epoch.
struct GnssEpoch
{
  GnssMode mode;
  // e = s + n (ENU, m) to add to the true antenna position; nothing in an
  // outage (no fix is published).
  std::optional<Eigen::Vector3d> error;
  // The true slow part s (ENU, m), for the evaluation; zero in an outage.
  Eigen::Vector3d slow_error = Eigen::Vector3d::Zero();
  // Per-axis total 1-sigma the receiver reports, sqrt(sigma_w^2 + sigma_ss^2)
  // (m); zero in an outage.
  Eigen::Vector3d sigma = Eigen::Vector3d::Zero();
};

class GnssCorruptor
{
public:
  GnssCorruptor(
    const GnssErrorParams & params, GnssSchedule schedule, std::uint64_t global_seed);

  // The epoch of a clean fix stamped stamp_ns (ns). The first epoch is t = 0
  // of the schedule. Returns nothing for a stamp not later than the last
  // one; such an epoch changes no state.
  std::optional<GnssEpoch> process(std::int64_t stamp_ns);

private:
  // The two error parts of one axis in one mode.
  struct Axis
  {
    // Streams "gnss.<mode>.white_<axis>" and "gnss.<mode>.slow_<axis>".
    Axis(double white_sigma, double slow_sigma, double drift_time, std::uint64_t seed,
      const std::string & mode, char axis);
    double white_sigma;
    RandomStream white;
    GaussMarkov slow;
  };
  using ModeAxes = std::array<Axis, 3>;  // E, N, U

  static ModeAxes make_axes(
    const GnssModeErrorParams & p, std::uint64_t seed, const std::string & mode);
  ModeAxes & axes_of(GnssMode mode);

  std::array<ModeAxes, 3> modes_;  // standalone, rtk_float, rtk_fixed
  GnssSchedule schedule_;
  std::optional<std::int64_t> first_stamp_ns_;
  std::optional<std::int64_t> last_stamp_ns_;
  // Mode and stamp of the last published fix (outages are skipped).
  std::optional<GnssMode> last_fix_mode_;
  std::int64_t last_fix_stamp_ns_ = 0;
};

}  // namespace asv_core
