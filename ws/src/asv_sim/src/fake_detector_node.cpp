// fake_detector: buoy measurements from ground truth (M3.1, D8 a,
// design.md section 5). Only I/O; the maths is asv_core::FakeDetector, the
// glue asv_sim::detector_setup() and friends (fake_detection.hpp).
//   in:  /asv/ground_truth/odom    true pose of base_link, 50 Hz
//        /asv/ground_truth/buoys   true map (latched, buoy_map_publisher)
//   out: /asv/perception/buoys     asv_msgs/BuoyDetectionArray, one per scan
//                                  (10 Hz = lidar rate), also when empty
// A scan is taken at the first odometry stamp of each scan period, with that
// message's pose (asv_core::ScanGrid, decision M3.1 A). Stamp = odometry stamp.
// Odometry that arrives before the map gives no scan.
// Started by sim.launch.py (launch arguments fake_detector:=false,
// detector:=<name>). Alone:
//      ros2 run asv_sim fake_detector --ros-args -p use_sim_time:=true
//        [-p detector:=<name>]   (config/detector/<name>.yaml)
//
// asv.yaml, asv_frames.yaml, asv_report.json and the detector file are read
// from the package share directory, like the corruptor.
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "asv_sim/fake_detection.hpp"

class FakeDetectorNode : public rclcpp::Node
{
public:
  FakeDetectorNode()
  : Node("fake_detector")
  {
    const std::string name = declare_parameter<std::string>("detector", "nominal");
    const std::string config =
      ament_index_cpp::get_package_share_directory("asv_sim") + "/config";
    setup_ = asv_sim::detector_setup(
      YAML::LoadFile(config + "/asv.yaml"), YAML::LoadFile(config + "/asv_frames.yaml"),
      YAML::LoadFile(config + "/asv_report.json"),
      YAML::LoadFile(config + "/detector/" + name + ".yaml"));
    grid_ = std::make_unique<asv_core::ScanGrid>(setup_.scan_period_ns);
    const auto & p = setup_.params;
    RCLCPP_INFO(
      get_logger(),
      "detector '%s', seed %lu, scan every %.3f s; sigma_r %.4g m, sigma_beta %.4g rad, "
      "P_miss %.3g, lambda %.3g, p_c %.3g, centroid bias %s; %zu cameras",
      name.c_str(), static_cast<unsigned long>(setup_.seed), setup_.scan_period_ns * 1e-9,
      p.range_sigma, p.bearing_sigma, p.miss_probability, p.false_alarm_rate,
      p.colour_error_probability, p.centroid_bias ? "on" : "off", p.cameras.size());

    pub_ = create_publisher<asv_msgs::msg::BuoyDetectionArray>(
      "/asv/perception/buoys", rclcpp::QoS(10));
    map_sub_ = create_subscription<asv_msgs::msg::LandmarkArray>(
      "/asv/ground_truth/buoys", rclcpp::QoS(1).reliable().transient_local(),
      [this](const asv_msgs::msg::LandmarkArray & msg) {on_map(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/asv/ground_truth/odom", rclcpp::QoS(50),
      [this](const nav_msgs::msg::Odometry & msg) {on_odom(msg);});
  }

private:
  void on_map(const asv_msgs::msg::LandmarkArray & map)
  {
    if (detector_) {
      return;  // the map is static (D7 a): the first copy is kept
    }
    detector_ = std::make_unique<asv_core::FakeDetector>(
      setup_.params, asv_sim::detector_objects(map, setup_.limits), setup_.seed);
    RCLCPP_INFO(get_logger(), "map with %zu objects received", map.landmarks.size());
  }

  void on_odom(const nav_msgs::msg::Odometry & odom)
  {
    if (!detector_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no buoy map yet: no scan");
      return;
    }
    if (!grid_->due(asv_sim::stamp_nanoseconds(odom.header.stamp))) {
      return;
    }
    const auto detections = detector_->scan(asv_sim::pose_from_odometry(odom));
    pub_->publish(asv_sim::detection_message(detections, odom.header.stamp));
  }

  asv_sim::DetectorSetup setup_;
  std::unique_ptr<asv_core::ScanGrid> grid_;
  std::unique_ptr<asv_core::FakeDetector> detector_;
  rclcpp::Publisher<asv_msgs::msg::BuoyDetectionArray>::SharedPtr pub_;
  rclcpp::Subscription<asv_msgs::msg::LandmarkArray>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
