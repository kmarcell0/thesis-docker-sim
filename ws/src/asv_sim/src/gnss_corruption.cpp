// GNSS corruption glue (M2.6b); see gnss_corruption.hpp.
#include "asv_sim/gnss_corruption.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "asv_core/geodesy.hpp"

namespace asv_sim
{

namespace
{

// The sensor_specs entry of the one sensor with type gnss or gnss_dual.
YAML::Node gnss_spec(const YAML::Node & asv)
{
  YAML::Node spec;
  int count = 0;
  for (const auto & s : asv["sensors"]) {
    const std::string type = s["type"].as<std::string>();
    if (type == "gnss" || type == "gnss_dual") {
      spec = asv["sensor_specs"][s["spec"].as<std::string>()];
      ++count;
    }
  }
  if (count != 1) {
    throw std::invalid_argument(
            "asv.yaml must have exactly one GNSS sensor, found " + std::to_string(count));
  }
  return spec;
}

asv_core::GnssModeErrorParams mode_params(
  const YAML::Node & table, const char * mode, double white, double slow)
{
  const YAML::Node m = table[mode];
  asv_core::GnssModeErrorParams p;
  p.white_h = m["white_h"].as<double>() * white;
  p.slow_h = m["slow_h"].as<double>() * slow;
  p.white_v = m["white_v"].as<double>() * white;
  p.slow_v = m["slow_v"].as<double>() * slow;
  p.drift_time = m["drift_time"].as<double>();
  return p;
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(t.nanosec);
}

}  // namespace

asv_core::GnssErrorParams gnss_error_params(const YAML::Node & asv, const YAML::Node & profile)
{
  const YAML::Node table = gnss_spec(asv)["position_modes"];
  const double white = profile["gnss"]["white"].as<double>();
  const double slow = profile["gnss"]["slow"].as<double>();
  asv_core::GnssErrorParams p;
  p.standalone = mode_params(table, "standalone", white, slow);
  p.rtk_float = mode_params(table, "rtk_float", white, slow);
  p.rtk_fixed = mode_params(table, "rtk_fixed", white, slow);
  return p;
}

asv_core::GnssSchedule gnss_schedule(const YAML::Node & profile)
{
  std::vector<asv_core::GnssScheduleEntry> entries;
  for (const auto & e : profile["gnss"]["schedule"]) {
    entries.push_back(
      {e["from"].as<double>(), asv_core::gnss_mode_from_name(e["mode"].as<std::string>())});
  }
  return asv_core::GnssSchedule(std::move(entries));
}

std::optional<GnssMessages> corrupt_gnss(
  const sensor_msgs::msg::NavSatFix & clean, asv_core::GnssCorruptor & corruptor)
{
  const auto ep = corruptor.process(stamp_ns(clean.header.stamp));
  if (!ep) {
    return std::nullopt;
  }

  GnssMessages out;
  out.status.header.stamp = clean.header.stamp;  // frame_id empty (GnssStatus.msg)
  out.status.position_mode = static_cast<std::uint8_t>(ep->mode);
  out.status.sky_view_outage = ep->mode == asv_core::GnssMode::outage;
  out.status.corrections_lost = ep->mode == asv_core::GnssMode::standalone;
  out.status.heading_available = false;  // no heading output before M2.6c
  if (!ep->error) {
    return out;  // outage: no fix
  }

  // The error is added in an ENU frame centred on the clean fix. Its axes
  // differ from the world's by (distance from the world origin) / (Earth
  // radius): at 100 m that turns a 1.5 m error by 24 um, negligible.
  const asv_core::LocalTangentPlane here({clean.latitude, clean.longitude, clean.altitude});
  const asv_core::GeoPoint p = here.to_geo(*ep->error);

  sensor_msgs::msg::NavSatFix fix;
  fix.header = clean.header;
  fix.status.service = clean.status.service;
  fix.status.status = ep->mode == asv_core::GnssMode::standalone ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  fix.latitude = p.lat_deg;
  fix.longitude = p.lon_deg;
  fix.altitude = p.alt;
  fix.position_covariance.fill(0.0);
  for (int j = 0; j < 3; ++j) {
    fix.position_covariance[static_cast<std::size_t>(4 * j)] = ep->sigma[j] * ep->sigma[j];
  }
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  out.fix = fix;

  geometry_msgs::msg::Vector3Stamped s;
  s.header.stamp = clean.header.stamp;
  s.header.frame_id = "world";
  s.vector.x = ep->slow_error.x();
  s.vector.y = ep->slow_error.y();
  s.vector.z = ep->slow_error.z();
  out.slow_error = s;
  return out;
}

}  // namespace asv_sim
