// Glue between the corruptor node and asv_core::GnssCorruptor (M2.6b): reads
// the GNSS error parameters from asv.yaml and the mode schedule from a noise
// profile, and turns one clean sensor_msgs/NavSatFix into the corrupted fix
// and the ground-truth messages. No node here, so it is unit tested
// (design.md section 4.2).
#pragma once

#include <optional>

#include "asv_core/gnss_corruptor.hpp"
#include "asv_msgs/msg/gnss_status.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "yaml-cpp/yaml.h"

namespace asv_sim
{

// Error parameters of the one sensor with type gnss or gnss_dual in
// asv.yaml: its sensor_specs position_modes table, white sizes times the
// profile's gnss.white and slow sizes times gnss.slow. A missing key throws
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
};

// Corrupts one clean fix. Nothing for a stamp not later than the previous
// one (GnssCorruptor::process()).
std::optional<GnssMessages> corrupt_gnss(
  const sensor_msgs::msg::NavSatFix & clean, asv_core::GnssCorruptor & corruptor);

}  // namespace asv_sim
