// Fake detector glue (M3.1): builds asv_core::FakeDetector from the boat
// description and the detector settings, and converts between ROS messages
// and the ROS-free types. Message glue only (no node), so it is unit tested;
// the node fake_detector does the I/O.
//
// Where each number comes from (extrinsics never typed into code):
//   lidar and camera positions, camera yaw   asv_frames.yaml (<name>_link)
//   camera hfov, lidar rate (scan period)    asv.yaml sensor_specs
//   range limits per class                   asv_report.json: lidar
//                                            max_distance_2_beams_m, cameras
//                                            max_distance_20_px_m ("<class> buoy")
//   classes without a report row             detector yaml class_limits_as
//   noise, misses, false alarms, mix-ups     detector yaml (config/detector/)
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "asv_core/fake_detector.hpp"
#include "asv_msgs/msg/buoy_detection_array.hpp"
#include "asv_msgs/msg/landmark_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"

namespace asv_sim
{

// Range limits of one object class.
struct ClassLimits
{
  double lidar = 0.0;           // [m]
  std::vector<double> camera;   // [m] one per camera, same order as the cameras
};

struct DetectorSetup
{
  asv_core::DetectorParams params;
  std::map<std::string, ClassLimits> limits;  // by buoys_gt.yaml class name
  std::uint64_t seed = 0;
  std::int64_t scan_period_ns = 0;
  std::vector<std::string> camera_names;      // for the log
};

// asv: asv.yaml, frames: asv_frames.yaml, report: asv_report.json (yaml-cpp
// reads JSON), detector: config/detector/<name>.yaml. Throws
// std::runtime_error (or YAML::Exception) on a missing key or sensor.
DetectorSetup detector_setup(
  const YAML::Node & asv, const YAML::Node & frames, const YAML::Node & report,
  const YAML::Node & detector);

// The objects of the true map with their class limits. Class "other" is a
// vessel. Throws std::runtime_error for a class without limits.
std::vector<asv_core::DetectorObject> detector_objects(
  const asv_msgs::msg::LandmarkArray & map, const std::map<std::string, ClassLimits> & limits);

// Planar pose of base_link from the ground-truth odometry (yaw from the
// quaternion; roll and pitch dropped).
asv_core::Pose2 pose_from_odometry(const nav_msgs::msg::Odometry & odom);

// One scan as a message: frame base_link, the given stamp; true ids dropped.
asv_msgs::msg::BuoyDetectionArray detection_message(
  const std::vector<asv_core::Detection> & detections,
  const builtin_interfaces::msg::Time & stamp);

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & t);

}  // namespace asv_sim
