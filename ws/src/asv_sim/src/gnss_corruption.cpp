// GNSS corruption glue (M2.6b position, M2.6c velocity and heading); see
// gnss_corruption.hpp.
#include "asv_sim/gnss_corruption.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "asv_core/angles.hpp"
#include "asv_core/geodesy.hpp"

namespace asv_sim
{

namespace
{

// The sensors entry of the one sensor with type gnss or gnss_dual.
YAML::Node gnss_sensor(const YAML::Node & asv)
{
  YAML::Node sensor;
  int count = 0;
  for (const auto & s : asv["sensors"]) {
    const std::string type = s["type"].as<std::string>();
    if (type == "gnss" || type == "gnss_dual") {
      sensor.reset(s);  // alias the entry (operator= would convert)
      ++count;
    }
  }
  if (count != 1) {
    throw std::invalid_argument(
            "asv.yaml must have exactly one GNSS sensor, found " + std::to_string(count));
  }
  return sensor;
}

Eigen::Vector3d vec3(const YAML::Node & n)
{
  return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
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
  const YAML::Node sensor = gnss_sensor(asv);
  const YAML::Node spec = asv["sensor_specs"][sensor["spec"].as<std::string>()];
  const YAML::Node table = spec["position_modes"];
  const YAML::Node g = profile["gnss"];
  const double white = g["white"].as<double>();
  const double slow = g["slow"].as<double>();
  asv_core::GnssErrorParams p;
  p.standalone = mode_params(table, "standalone", white, slow);
  p.rtk_float = mode_params(table, "rtk_float", white, slow);
  p.rtk_fixed = mode_params(table, "rtk_fixed", white, slow);
  p.velocity_white = spec["velocity_noise"].as<double>() * g["velocity"].as<double>();
  if (sensor["type"].as<std::string>() == "gnss_dual") {
    // D12 a: the datasheet heading accuracy is for a 1 m baseline and scales
    // with 1 / baseline.
    const double baseline = (vec3(sensor["xyz_secondary"]) - vec3(sensor["xyz"])).norm();
    p.heading_white = spec["heading_accuracy_1m"].as<double>() * M_PI / 180.0 / baseline *
      g["heading"].as<double>();
  }
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
  out.epoch = *ep;
  out.status.header.stamp = clean.header.stamp;  // frame_id empty (GnssStatus.msg)
  out.status.position_mode = static_cast<std::uint8_t>(ep->mode);
  out.status.sky_view_outage = ep->mode == asv_core::GnssMode::outage;
  out.status.corrections_lost = ep->mode == asv_core::GnssMode::standalone;
  out.status.heading_available = false;  // set by GnssCorruption (M2.6c)
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

GnssCorruption::GnssCorruption(
  const asv_core::GnssErrorParams & params, asv_core::GnssSchedule schedule,
  std::uint64_t global_seed)
: corruptor_(params, std::move(schedule), global_seed),
  velocity_sigma_(params.velocity_white),
  heading_sigma_(params.heading_white.value_or(0.0)),
  yaw_(2'000'000'000LL)  // 2 s of odometry (100 samples at 50 Hz)
{
}

GnssOutputs GnssCorruption::on_fix(const sensor_msgs::msg::NavSatFix & clean)
{
  GnssOutputs out;
  out.position = corrupt_gnss(clean, corruptor_);
  if (!out.position) {
    return out;  // old stamp: no new epoch
  }
  const asv_core::GnssEpoch & ep = out.position->epoch;
  out.position->status.heading_available = ep.heading_noise.has_value();

  // Velocity of the middle epoch k-1 from the clean fixes k-2 and k (Q1 a).
  // The clean fixes keep coming in an outage, so only the noise says whether
  // epoch k-1 had a velocity.
  if (before_ && middle_ && middle_->velocity_noise) {
    const asv_core::LocalTangentPlane here(
      {middle_->fix.latitude, middle_->fix.longitude, middle_->fix.altitude});
    const auto enu = [&here](const sensor_msgs::msg::NavSatFix & f) {
        return here.to_enu({f.latitude, f.longitude, f.altitude});
      };
    const Eigen::Vector3d v = asv_core::central_difference(
      enu(before_->fix), stamp_ns(before_->fix.header.stamp),
      enu(clean), stamp_ns(clean.header.stamp)) + *middle_->velocity_noise;
    geometry_msgs::msg::TwistWithCovarianceStamped m;
    m.header.stamp = middle_->fix.header.stamp;
    m.header.frame_id = "world";
    m.twist.twist.linear.x = v.x();
    m.twist.twist.linear.y = v.y();
    m.twist.twist.linear.z = v.z();
    m.twist.covariance.fill(0.0);
    for (std::size_t j = 0; j < 3; ++j) {
      m.twist.covariance[7 * j] = velocity_sigma_ * velocity_sigma_;
      m.twist.covariance[7 * (j + 3)] = -1.0;  // angular velocity not measured
    }
    out.velocity = m;
  }
  before_ = std::move(middle_);
  middle_ = CleanEpoch{clean, ep.velocity_noise};

  if (ep.heading_noise) {
    pending_.push_back({clean.header.stamp, stamp_ns(clean.header.stamp), *ep.heading_noise});
  }
  out.headings = ready_headings();
  return out;
}

std::vector<asv_msgs::msg::GnssHeading> GnssCorruption::on_odom(
  const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  yaw_.add(stamp_ns(odom.header.stamp), asv_core::yaw_from_quaternion(q.w, q.x, q.y, q.z));
  return ready_headings();
}

std::vector<asv_msgs::msg::GnssHeading> GnssCorruption::ready_headings()
{
  // At most 5 s of epochs wait for their yaw (odometry missing or late);
  // older ones are dropped.
  constexpr std::size_t kMaxPending = 50;
  while (pending_.size() > kMaxPending) {
    pending_.pop_front();
    ++dropped_headings_;
  }
  std::vector<asv_msgs::msg::GnssHeading> ready;
  while (!pending_.empty()) {
    const PendingHeading & p = pending_.front();
    const auto psi = yaw_.at(p.stamp_ns);
    if (!psi) {
      const auto oldest = yaw_.oldest();
      if (oldest && p.stamp_ns < *oldest) {
        pending_.pop_front();  // its yaw is gone: never ready
        ++dropped_headings_;
        continue;
      }
      break;  // odometry up to this stamp not here yet
    }
    asv_msgs::msg::GnssHeading h;
    h.header.stamp = p.stamp;
    h.header.frame_id = "base_link";
    // alpha = wrap(pi/2 - psi + sigma_alpha w) = azimuth_from_yaw(psi - noise)
    h.azimuth = asv_core::azimuth_from_yaw(*psi - p.noise);
    h.azimuth_std = heading_sigma_;
    h.baseline_fixed = true;
    ready.push_back(h);
    pending_.pop_front();
  }
  return ready;
}

}  // namespace asv_sim
