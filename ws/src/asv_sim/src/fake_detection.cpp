// Fake detector glue (M3.1); see fake_detection.hpp.
#include "asv_sim/fake_detection.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "asv_core/angles.hpp"
#include "asv_msgs/msg/buoy_colour.hpp"
#include "asv_msgs/msg/buoy_detection.hpp"

namespace asv_sim
{

namespace
{

using asv_msgs::msg::BuoyColour;
using asv_msgs::msg::BuoyDetection;
static_assert(
  BuoyColour::UNKNOWN == static_cast<std::uint8_t>(asv_core::BuoyColour::unknown) &&
  BuoyColour::RED == static_cast<std::uint8_t>(asv_core::BuoyColour::red) &&
  BuoyColour::GREEN == static_cast<std::uint8_t>(asv_core::BuoyColour::green) &&
  BuoyColour::YELLOW == static_cast<std::uint8_t>(asv_core::BuoyColour::yellow) &&
  BuoyColour::BLACK == static_cast<std::uint8_t>(asv_core::BuoyColour::black),
  "asv_core::BuoyColour must match asv_msgs/BuoyColour");

// The asv_frames.yaml entry base_link -> <child>.
YAML::Node base_link_frame(const YAML::Node & frames, const std::string & child)
{
  for (const auto & f : frames["frames"]) {
    if (f["parent"].as<std::string>() == "base_link" && f["child"].as<std::string>() == child) {
      return f;
    }
  }
  throw std::runtime_error("asv_frames.yaml has no frame base_link -> " + child);
}

// Limit of a class in a report table keyed "<class> buoy".
double class_limit(const YAML::Node & table, const std::string & cls, const std::string & where)
{
  const YAML::Node v = table[cls + " buoy"];
  if (!v) {
    throw std::runtime_error(where + " has no entry '" + cls + " buoy'");
  }
  return v.as<double>();
}

}  // namespace

DetectorSetup detector_setup(
  const YAML::Node & asv, const YAML::Node & frames, const YAML::Node & report,
  const YAML::Node & detector)
{
  DetectorSetup s;
  auto & p = s.params;

  // Sensors in asv.yaml order: one lidar, any number of cameras.
  std::string lidar_name;
  int lidars = 0;
  for (const auto & sensor : asv["sensors"]) {
    const std::string type = sensor["type"].as<std::string>();
    const std::string name = sensor["name"].as<std::string>();
    const YAML::Node spec = asv["sensor_specs"][sensor["spec"].as<std::string>()];
    if (type == "lidar") {
      ++lidars;
      lidar_name = name;
      const YAML::Node xyz = base_link_frame(frames, name + "_link")["xyz"];
      p.lidar_x = xyz[0].as<double>();
      p.lidar_y = xyz[1].as<double>();
      s.scan_period_ns = static_cast<std::int64_t>(std::llround(1e9 / spec["rate"].as<double>()));
    } else if (type == "camera" || type == "stereo_camera") {
      const YAML::Node f = base_link_frame(frames, name + "_link");
      asv_core::DetectorCamera cam;
      cam.x = f["xyz"][0].as<double>();
      cam.y = f["xyz"][1].as<double>();
      cam.yaw = f["rpy"][2].as<double>();
      cam.hfov = spec["hfov"].as<double>();
      p.cameras.push_back(cam);
      s.camera_names.push_back(name);
    }
  }
  if (lidars != 1) {
    throw std::runtime_error(
            "asv.yaml must have exactly one lidar, found " + std::to_string(lidars));
  }

  // Limits of the classes the report knows.
  const YAML::Node sensors = report["sensors"];
  const YAML::Node lidar_table = sensors[lidar_name]["max_distance_2_beams_m"];
  for (const char * cls : {"A-0", "A-2", "Sur-Mark"}) {
    ClassLimits l;
    l.lidar = class_limit(lidar_table, cls, "asv_report.json " + lidar_name);
    for (const auto & cam : s.camera_names) {
      l.camera.push_back(
        class_limit(
          sensors[cam]["max_distance_20_px_m"], cls, "asv_report.json " + cam));
    }
    s.limits[cls] = l;
  }
  // The others borrow another class's limits (detector yaml, [EST]).
  for (const auto & kv : detector["class_limits_as"]) {
    const std::string cls = kv.first.as<std::string>();
    const std::string as = kv.second.as<std::string>();
    const auto it = s.limits.find(as);
    if (it == s.limits.end()) {
      throw std::runtime_error("class_limits_as: '" + cls + "' uses unknown class '" + as + "'");
    }
    s.limits[cls] = it->second;
  }

  s.seed = detector["seed"].as<std::uint64_t>();
  p.range_sigma = detector["range_sigma"].as<double>();
  p.bearing_sigma = detector["bearing_sigma"].as<double>();
  p.miss_probability = detector["miss_probability"].as<double>();
  p.false_alarm_rate = detector["false_alarm_rate"].as<double>();
  p.false_alarm_max_range = detector["false_alarm_max_range"].as<double>();
  p.colour_error_probability = detector["colour_error_probability"].as<double>();
  p.centroid_bias = detector["centroid_bias"].as<bool>();
  return s;
}

std::vector<asv_core::DetectorObject> detector_objects(
  const asv_msgs::msg::LandmarkArray & map, const std::map<std::string, ClassLimits> & limits)
{
  std::vector<asv_core::DetectorObject> out;
  for (const auto & l : map.landmarks) {
    const auto it = limits.find(l.model);
    if (it == limits.end()) {
      throw std::runtime_error("no range limits for class '" + l.model + "' (" + l.name + ")");
    }
    asv_core::DetectorObject o;
    o.id = l.id;
    o.x = l.x;
    o.y = l.y;
    o.radius = l.radius;
    o.colour = static_cast<asv_core::BuoyColour>(l.colour.value);
    o.vessel = l.model == "other";
    o.lidar_range = it->second.lidar;
    o.camera_range = it->second.camera;
    out.push_back(o);
  }
  return out;
}

asv_core::Pose2 pose_from_odometry(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  return asv_core::Pose2{
    odom.pose.pose.position.x, odom.pose.pose.position.y,
    asv_core::yaw_from_quaternion(q.w, q.x, q.y, q.z)};
}

asv_msgs::msg::BuoyDetectionArray detection_message(
  const std::vector<asv_core::Detection> & detections,
  const builtin_interfaces::msg::Time & stamp)
{
  asv_msgs::msg::BuoyDetectionArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  for (const auto & d : detections) {
    BuoyDetection m;
    m.range = d.range;
    m.bearing = d.bearing;
    m.colour.value = static_cast<std::uint8_t>(d.colour);
    m.object_class = d.vessel ? BuoyDetection::CLASS_OTHER : BuoyDetection::CLASS_BUOY;
    msg.detections.push_back(m);
  }
  return msg;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1'000'000'000LL + t.nanosec;
}

}  // namespace asv_sim
