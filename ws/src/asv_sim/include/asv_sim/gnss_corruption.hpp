// Glue between the corruptor node and asv_core::GnssCorruptor: reads the
// GNSS error parameters from asv.yaml and the mode schedule from a noise
// profile, and turns the clean sensor_msgs/NavSatFix (and the ground-truth
// yaw) into the corrupted fix (M2.6b), velocity and heading (M2.6c) and the
// ground-truth messages. No node here, so it is unit tested (design.md
// sections 4.2, 4.3).
#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

#include "asv_core/gnss_corruptor.hpp"
#include "asv_msgs/msg/gnss_heading.hpp"
#include "asv_msgs/msg/gnss_status.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "yaml-cpp/yaml.h"

namespace asv_sim
{

// Error parameters of the one sensor with type gnss or gnss_dual in
// asv.yaml: its sensor_specs position_modes table, white sizes times the
// profile's gnss.white and slow sizes times gnss.slow; velocity_noise times
// gnss.velocity; for gnss_dual the heading sigma
//   sigma_alpha = heading_accuracy_1m [deg -> rad] / baseline [m]   (D12 a)
// with the baseline = |xyz_secondary - xyz| of the sensor entry, times
// gnss.heading (type gnss: no heading). A missing key throws
// (YAML::Exception); no GNSS or more than one throws std::invalid_argument.
asv_core::GnssErrorParams gnss_error_params(const YAML::Node & asv, const YAML::Node & profile);

// The profile's gnss.schedule: a list of {from: <s>, mode: <name>}, from
// counted from the first clean fix. Throws std::invalid_argument for an
// unknown mode or a bad order (asv_core::GnssSchedule).
asv_core::GnssSchedule gnss_schedule(const YAML::Node & profile);

struct GnssMessages
{
  // /asv/sensors/gnss/fix: the clean antenna fix + error, same header.
  // position_covariance = diag of the total per-axis sigma^2 (ENU, m^2),
  // DIAGONAL_KNOWN; status FIX in standalone, GBAS_FIX with RTK (NavSatStatus
  // can't tell float from fixed: the covariance says it, assumption A4).
  // Nothing in an outage.
  std::optional<sensor_msgs::msg::NavSatFix> fix;
  // /asv/ground_truth/gnss_status: the true mode, every epoch.
  asv_msgs::msg::GnssStatus status;
  // /asv/ground_truth/gnss_error: the true slow error s (ENU, m), frame
  // world. Nothing in an outage.
  std::optional<geometry_msgs::msg::Vector3Stamped> slow_error;
  // The epoch's draws, incl. the velocity and heading noise (M2.6c), which
  // GnssCorruption turns into messages later.
  asv_core::GnssEpoch epoch;
};

// Corrupts one clean fix. Nothing for a stamp not later than the previous
// one (GnssCorruptor::process()).
std::optional<GnssMessages> corrupt_gnss(
  const sensor_msgs::msg::NavSatFix & clean, asv_core::GnssCorruptor & corruptor);

// What one clean message gives (M2.6c). Each part may be empty.
struct GnssOutputs
{
  // Fix, status and slow error of this epoch (as corrupt_gnss()); nothing for
  // an old stamp.
  std::optional<GnssMessages> position;
  // /asv/sensors/gnss/velocity of the PREVIOUS epoch k-1 (central
  // difference needs fix k): stamp t_{k-1}, frame "world" (ENU), the
  // primary antenna's velocity + noise (m/s). twist.covariance: linear diag
  // sigma_v^2, angular diag -1 (not measured).
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> velocity;
  // /asv/sensors/gnss/heading of every waiting epoch whose true yaw is now
  // known, oldest first.
  std::vector<asv_msgs::msg::GnssHeading> headings;
};

// The whole GNSS output of the corruptor (design.md 4.2, 4.3; M2.6c Q1 a,
// Q2 a, 2026-11-02):
// - position: corrupt_gnss() of each clean fix;
// - velocity of epoch k: v_k = central_difference(p_{k-1}, p_{k+1}) of the
//   clean antenna fixes, in an ENU frame centred on fix k (its axes differ
//   from the world's by distance / Earth radius: 1.6e-5 rad at 100 m), plus
//   the noise drawn at epoch k; given out when fix k+1 arrives;
// - heading of epoch k: azimuth_from_yaw(psi(t_k) - noise) =
//   wrap_[0,2pi)(pi/2 - psi(t_k) + sigma_alpha w), psi = yaw of the
//   ground-truth odometry at t_k (YawHistory); waits until odometry up to t_k
//   has arrived.
// Nothing for an epoch in a sky-view outage.
class GnssCorruption
{
public:
  GnssCorruption(
    const asv_core::GnssErrorParams & params, asv_core::GnssSchedule schedule,
    std::uint64_t global_seed);

  // A clean primary antenna fix (/asv/gnss/fix).
  GnssOutputs on_fix(const sensor_msgs::msg::NavSatFix & clean);

  // A ground-truth odometry message (/asv/ground_truth/odom): only its
  // stamp and yaw are used. Returns the headings that became ready.
  std::vector<asv_msgs::msg::GnssHeading> on_odom(const nav_msgs::msg::Odometry & odom);

  // Headings dropped because their yaw was no longer (or never) available.
  std::size_t dropped_headings() const {return dropped_headings_;}

private:
  struct PendingHeading
  {
    builtin_interfaces::msg::Time stamp;
    std::int64_t stamp_ns;
    double noise;
  };
  struct CleanEpoch
  {
    sensor_msgs::msg::NavSatFix fix;
    std::optional<Eigen::Vector3d> velocity_noise;
  };

  std::vector<asv_msgs::msg::GnssHeading> ready_headings();

  asv_core::GnssCorruptor corruptor_;
  double velocity_sigma_;
  double heading_sigma_;
  asv_core::YawHistory yaw_;
  std::deque<PendingHeading> pending_;
  std::optional<CleanEpoch> before_, middle_;  // epochs k-2 and k-1
  std::size_t dropped_headings_ = 0;
};

}  // namespace asv_sim
