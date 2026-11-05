// Fake detector, ROS-free (M3.1, D8 a): buoy measurements made from the true
// boat pose and the true buoy map, with seeded noise, misses, false alarms
// and colour mix-ups. Stands in for lidar clustering + camera colour (M5).
// Thesis: "Experimental Scenarios" (design.md section 5).
// Code -> glossary symbols: range r, bearing beta, sigma_r, sigma_beta,
// P_miss, lambda (false alarms per scan), p_c (colour mix-up), r_b (buoy
// radius), psi (yaw).
//
// Per scan, for every map object j (position p_j, radius r_b,j):
//   lidar sees j      if || p_j - p_lidar || <= lidar range limit of j's class
//   colour of j known if j is inside a camera's horizontal field of view and
//                     || p_j - p_cam || <= that camera's limit for the class,
//                     else UNKNOWN
//   r~    = || p_j - p || - c * (pi/4) * r_b,j + n_r      n_r ~ N(0, sigma_r^2)
//   beta~ = wrap( atan2(dy, dx) - psi + n_beta )         n_beta ~ N(0, sigma_beta^2)
// p = base_link origin (the message is measured from there, BuoyDetection.msg),
// c = 1 with the centroid bias on: lidar points spread evenly across the
// buoy's width w in [-r_b, r_b] lie at depth sqrt(r_b^2 - w^2) in front of
// the centre, mean (pi/4) r_b, so their centroid is that much nearer.
// Each seen object is then missed with probability P_miss; a known colour is
// swapped for one of the other three with probability p_c; Poisson(lambda)
// false alarms are added (bearing uniform in (-pi, pi], range uniform in
// (0, false_alarm_max_range], colour UNKNOWN, class buoy).
// Occlusion (a buoy behind another) is ignored: a stated limitation.
// Roll and pitch are ignored: r and beta are in the level frame by
// construction, since the pose is planar (x, y, psi).
//
// Random streams (noise.hpp), one per effect, so switching one effect on or
// off never changes the numbers of another ("common random numbers"):
//   "detector.range", "detector.bearing"  one draw per seen object
//   "detector.miss"                       one draw per seen object
//   "detector.colour"                     two draws per seen, coloured object
//   "detector.false_alarm"                count and positions
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "asv_core/noise.hpp"

namespace asv_core
{

// Colour codes: the numbers of asv_msgs/BuoyColour (checked in the node).
enum class BuoyColour : std::uint8_t
{
  unknown = 0,
  red = 1,
  green = 2,
  yellow = 3,
  black = 4,
};

// Planar pose of base_link in the map (= world) frame.
struct Pose2
{
  double x = 0.0;    // [m] east
  double y = 0.0;    // [m] north
  double yaw = 0.0;  // [rad] psi, counter-clockwise from east
};

// One camera, as far as the detector needs it: position and yaw of its frame
// in base_link (pitch ignored: both cameras see the horizon) and its
// horizontal field of view.
struct DetectorCamera
{
  double x = 0.0;     // [m] in base_link
  double y = 0.0;     // [m]
  double yaw = 0.0;   // [rad] optical axis direction in base_link
  double hfov = 0.0;  // [rad] full horizontal field of view
};

// One object of the true map with the limits of its class already looked up.
struct DetectorObject
{
  std::uint32_t id = 0;   // true id (buoys_gt.yaml), never in the filter's message
  double x = 0.0;         // [m] in map
  double y = 0.0;         // [m]
  double radius = 0.0;    // [m] r_b
  BuoyColour colour = BuoyColour::unknown;
  bool vessel = false;    // class "other": reported, never mapped
  double lidar_range = 0.0;           // [m] largest distance with >= 2 beams
  std::vector<double> camera_range;   // [m] per camera (same order), >= 20 px wide
};

struct DetectorParams
{
  double lidar_x = 0.0;            // [m] lidar position in base_link
  double lidar_y = 0.0;            // [m]
  std::vector<DetectorCamera> cameras;
  double range_sigma = 0.0;        // [m] sigma_r, >= 0
  double bearing_sigma = 0.0;      // [rad] sigma_beta, >= 0
  double miss_probability = 0.0;   // [-] P_miss in [0, 1]
  double false_alarm_rate = 0.0;   // [1/scan] lambda >= 0
  double false_alarm_max_range = 0.0;  // [m] > 0 if lambda > 0
  double colour_error_probability = 0.0;  // [-] p_c in [0, 1]
  bool centroid_bias = false;      // c
};

// One detection. true_id is empty for a false alarm.
struct Detection
{
  double range = 0.0;    // [m] r~, >= 0
  double bearing = 0.0;  // [rad] beta~ in (-pi, pi], positive to the left
  BuoyColour colour = BuoyColour::unknown;
  bool vessel = false;
  std::optional<std::uint32_t> true_id;
};

class FakeDetector
{
public:
  // Throws std::invalid_argument on a parameter out of range, or an object
  // whose camera_range does not have one entry per camera.
  FakeDetector(DetectorParams params, std::vector<DetectorObject> objects, std::uint64_t seed);

  // One scan at the true pose. The detections are sorted by measured bearing
  // (not by id: the order must not tell the filter which buoy is which).
  std::vector<Detection> scan(const Pose2 & pose);

  const DetectorParams & params() const {return params_;}

private:
  BuoyColour mixed_up(BuoyColour colour);
  void add_false_alarms(std::vector<Detection> & out);

  DetectorParams params_;
  std::vector<DetectorObject> objects_;
  RandomStream range_stream_;
  RandomStream bearing_stream_;
  RandomStream miss_stream_;
  RandomStream colour_stream_;
  RandomStream false_alarm_stream_;
};

// Picks the scans out of the ground-truth odometry stream (decision M3.1 A,
// 2026-11-05): a scan happens at the first stamp that reaches the next
// multiple of the scan period, and that message's pose is used exactly (no
// interpolation). With odometry on a 20 ms grid and a 100 ms period this is
// exactly every fifth message, whatever the offset of the first stamp.
class ScanGrid
{
public:
  // period_ns > 0, else std::invalid_argument.
  explicit ScanGrid(std::int64_t period_ns);

  // True if a scan is due at stamp t_ns (and remembers it). Stamps in the
  // same period as the last scan, or older, give false.
  bool due(std::int64_t t_ns);

private:
  std::int64_t period_ns_;
  std::optional<std::int64_t> last_index_;
};

}  // namespace asv_core
