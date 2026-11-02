// GNSS corruptor, ROS-free: the error of one GNSS fix (M2.6b), and the noise
// of the GNSS velocity and dual-antenna heading of the same epoch (M2.6c).
// Thesis: "Noise Models", "Vehicle and Sensor Models", "Sensor Configuration"
// (design.md sections 4.2, 4.3).
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
//
// Velocity and heading (M2.6c, decided 2026-11-02 Q3 a, Q4 a): white noise
// only, one draw per epoch, the same size in every mode (Doppler and the
// antenna baseline need no RTK corrections); none in an outage:
//   v_j(k)   = v_true,j(k) + sigma_v * w        (ENU, m/s; streams
//                                               "gnss.velocity_<e|n|u>")
//   alpha(k) = wrap_[0,2pi)(pi/2 - psi_true(k) + sigma_alpha * w)
//                                               (rad; stream "gnss.heading")
// The true values come from outside (central_difference(), azimuth_from_yaw(),
// YawHistory below); GnssCorruptor only draws the noise, so the draws do not
// depend on when the truth arrives.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
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
  double velocity_white = 0.0;  // sigma_v per ENU axis (m/s), >= 0
  // sigma_alpha of the azimuth (rad), >= 0; nothing = single antenna, no
  // heading output.
  std::optional<double> heading_white;
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
  // sigma_v * w per ENU axis (m/s) to add to the true antenna velocity;
  // nothing in an outage.
  std::optional<Eigen::Vector3d> velocity_noise;
  // sigma_alpha * w (rad) to add to the true azimuth; nothing in an outage
  // or without a heading (single antenna).
  std::optional<double> heading_noise;
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
  double velocity_white_;
  std::array<RandomStream, 3> velocity_streams_;  // E, N, U
  std::optional<double> heading_white_;
  RandomStream heading_stream_;
  GnssSchedule schedule_;
  std::optional<std::int64_t> first_stamp_ns_;
  std::optional<std::int64_t> last_stamp_ns_;
  // Mode and stamp of the last published fix (outages are skipped).
  std::optional<GnssMode> last_fix_mode_;
  std::int64_t last_fix_stamp_ns_ = 0;
};

// Azimuth alpha of the bow (rad, clockwise from north, in [0, 2 pi)) for the
// yaw psi of base_link (rad, counter-clockwise from east, ENU):
//   alpha = wrap_[0,2pi)(pi/2 - psi)                        (design.md 4.3)
// Works for any input angle, also with the noise already added.
double azimuth_from_yaw(double psi);

// Velocity at the middle epoch from the positions before and after it
// (central difference):
//   v_k = (p_{k+1} - p_{k-1}) / (t_{k+1} - t_{k-1})
// p in m, t in ns. Exact for a path with constant acceleration; on a circle
// at speed v and turn rate omega the error is about v omega^2 dt^2 / 6. A
// one-sided slope (p_k - p_{k-1}) / dt is the velocity at t_k - dt/2 instead.
// Throws std::invalid_argument unless t_next > t_prev.
Eigen::Vector3d central_difference(
  const Eigen::Vector3d & p_prev, std::int64_t t_prev_ns,
  const Eigen::Vector3d & p_next, std::int64_t t_next_ns);

// The recent yaw of the boat (from ground-truth odometry), for the yaw at a
// GNSS epoch stamp. Linear interpolation between the two samples around the
// stamp, through wrap_angle(), so it works across +-pi.
class YawHistory
{
public:
  // Keeps the samples of the last `keep_ns` (ns) before the newest one.
  explicit YawHistory(std::int64_t keep_ns);

  // Adds a sample; one not later than the newest is ignored.
  void add(std::int64_t stamp_ns, double psi);

  // psi at stamp_ns; nothing if stamp_ns is after the newest sample (not yet
  // known) or before the oldest kept one.
  std::optional<double> at(std::int64_t stamp_ns) const;

  // Stamp of the newest / oldest kept sample; nothing while empty.
  std::optional<std::int64_t> newest() const;
  std::optional<std::int64_t> oldest() const;

private:
  std::int64_t keep_ns_;
  std::deque<std::pair<std::int64_t, double>> samples_;
};

}  // namespace asv_core
