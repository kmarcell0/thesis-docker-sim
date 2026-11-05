// Fake detector (M3.1); see fake_detector.hpp.
#include "asv_core/fake_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "asv_core/angles.hpp"

namespace asv_core
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

void check(bool ok, const std::string & what)
{
  if (!ok) {
    throw std::invalid_argument("FakeDetector: " + what);
  }
}

bool is_probability(double p)
{
  return p >= 0.0 && p <= 1.0;
}

// Position of a point given in base_link (bx, by) in the map: p + R(psi) b.
void to_map(const Pose2 & pose, double bx, double by, double & mx, double & my)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  mx = pose.x + c * bx - s * by;
  my = pose.y + s * bx + c * by;
}

// True if the object's colour can be seen by at least one camera: inside its
// horizontal field of view (|angle off the optical axis| <= hfov/2) and
// within its range for the object's class.
bool colour_seen(const DetectorParams & p, const DetectorObject & o, const Pose2 & pose)
{
  for (std::size_t i = 0; i < p.cameras.size(); ++i) {
    const DetectorCamera & cam = p.cameras[i];
    double cx = 0.0;
    double cy = 0.0;
    to_map(pose, cam.x, cam.y, cx, cy);
    const double dx = o.x - cx;
    const double dy = o.y - cy;
    if (std::hypot(dx, dy) > o.camera_range[i]) {
      continue;
    }
    const double off_axis = wrap_angle(std::atan2(dy, dx) - pose.yaw - cam.yaw);
    if (std::abs(off_axis) <= 0.5 * cam.hfov) {
      return true;
    }
  }
  return false;
}

}  // namespace

FakeDetector::FakeDetector(
  DetectorParams params, std::vector<DetectorObject> objects, std::uint64_t seed)
: params_(std::move(params)),
  objects_(std::move(objects)),
  range_stream_(seed, "detector.range"),
  bearing_stream_(seed, "detector.bearing"),
  miss_stream_(seed, "detector.miss"),
  colour_stream_(seed, "detector.colour"),
  false_alarm_stream_(seed, "detector.false_alarm")
{
  check(params_.range_sigma >= 0.0, "range_sigma must be >= 0");
  check(params_.bearing_sigma >= 0.0, "bearing_sigma must be >= 0");
  check(is_probability(params_.miss_probability), "miss_probability must be in [0, 1]");
  check(
    is_probability(params_.colour_error_probability),
    "colour_error_probability must be in [0, 1]");
  check(params_.false_alarm_rate >= 0.0, "false_alarm_rate must be >= 0");
  check(
    params_.false_alarm_rate == 0.0 || params_.false_alarm_max_range > 0.0,
    "false_alarm_max_range must be > 0 when false_alarm_rate > 0");
  for (const auto & cam : params_.cameras) {
    check(cam.hfov > 0.0 && cam.hfov < 2.0 * kPi, "camera hfov must be in (0, 2 pi)");
  }
  for (const auto & o : objects_) {
    check(o.radius > 0.0, "object " + std::to_string(o.id) + ": radius must be > 0");
    check(
      o.camera_range.size() == params_.cameras.size(),
      "object " + std::to_string(o.id) + ": one camera_range per camera needed");
  }
}

std::vector<Detection> FakeDetector::scan(const Pose2 & pose)
{
  double lx = 0.0;
  double ly = 0.0;
  to_map(pose, params_.lidar_x, params_.lidar_y, lx, ly);
  const double bias_factor = params_.centroid_bias ? 0.25 * kPi : 0.0;

  std::vector<Detection> out;
  for (const auto & o : objects_) {
    if (std::hypot(o.x - lx, o.y - ly) > o.lidar_range) {
      continue;
    }
    // All draws of a seen object happen whatever the settings are, so each
    // stream advances the same way in every run of the same trajectory.
    const double n_r = params_.range_sigma * range_stream_.normal();
    const double n_beta = params_.bearing_sigma * bearing_stream_.normal();
    const bool missed = miss_stream_.uniform() < params_.miss_probability;

    const double dx = o.x - pose.x;
    const double dy = o.y - pose.y;
    Detection d;
    d.range = std::max(0.0, std::hypot(dx, dy) - bias_factor * o.radius + n_r);
    d.bearing = wrap_angle(std::atan2(dy, dx) - pose.yaw + n_beta);
    d.vessel = o.vessel;
    d.true_id = o.id;
    if (colour_seen(params_, o, pose) && o.colour != BuoyColour::unknown) {
      d.colour = mixed_up(o.colour);
    }
    if (!missed) {
      out.push_back(d);
    }
  }
  add_false_alarms(out);
  std::sort(
    out.begin(), out.end(),
    [](const Detection & a, const Detection & b) {return a.bearing < b.bearing;});
  return out;
}

// With probability p_c, one of the three other colours (equally likely).
BuoyColour FakeDetector::mixed_up(BuoyColour colour)
{
  const double u_swap = colour_stream_.uniform();
  const double u_pick = colour_stream_.uniform();
  if (u_swap >= params_.colour_error_probability) {
    return colour;
  }
  const int true_code = static_cast<int>(colour);       // 1..4
  int pick = 1 + std::min(2, static_cast<int>(3.0 * u_pick));  // 1..3
  if (pick >= true_code) {
    ++pick;  // skip the true colour
  }
  return static_cast<BuoyColour>(pick);
}

// Poisson(lambda) count by multiplying uniforms (Knuth): the count is the
// number of uniforms whose running product stays above exp(-lambda). Fine
// for the small lambda used here (a few per scan at most).
void FakeDetector::add_false_alarms(std::vector<Detection> & out)
{
  if (params_.false_alarm_rate <= 0.0) {
    return;
  }
  const double limit = std::exp(-params_.false_alarm_rate);
  int count = 0;
  double product = false_alarm_stream_.uniform();
  while (product > limit) {
    ++count;
    product *= false_alarm_stream_.uniform();
  }
  for (int i = 0; i < count; ++i) {
    Detection d;
    // 1 - u is in (0, 1]: range in (0, max], bearing in (-pi, pi].
    d.range = (1.0 - false_alarm_stream_.uniform()) * params_.false_alarm_max_range;
    d.bearing = kPi - 2.0 * kPi * false_alarm_stream_.uniform();
    out.push_back(d);
  }
}

ScanGrid::ScanGrid(std::int64_t period_ns)
: period_ns_(period_ns)
{
  if (period_ns <= 0) {
    throw std::invalid_argument("ScanGrid: period must be > 0");
  }
}

bool ScanGrid::due(std::int64_t t_ns)
{
  // Floor division, also correct for negative stamps.
  std::int64_t index = t_ns / period_ns_;
  if (t_ns % period_ns_ != 0 && t_ns < 0) {
    --index;
  }
  if (last_index_ && index <= *last_index_) {
    return false;
  }
  last_index_ = index;
  return true;
}

}  // namespace asv_core
