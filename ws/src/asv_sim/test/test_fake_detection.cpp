// Tests of the fake detector glue (M3.1): limits and geometry read from the
// repository config files, map and message conversion, and one scan at the
// spawn with the real map.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "asv_sim/buoy_map.hpp"
#include "asv_sim/fake_detection.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string config(const std::string & file)
{
  return std::string(ASV_CONFIG_DIR) + "/" + file;
}

asv_sim::DetectorSetup repo_setup()
{
  return asv_sim::detector_setup(
    YAML::LoadFile(config("asv.yaml")), YAML::LoadFile(config("asv_frames.yaml")),
    YAML::LoadFile(config("asv_report.json")), YAML::LoadFile(config("detector/nominal.yaml")));
}

// The true map as buoy_map_publisher sends it.
asv_msgs::msg::LandmarkArray repo_map_message()
{
  const auto map = asv_sim::parse_buoy_map(YAML::LoadFile(config("buoys_gt.yaml")));
  asv_msgs::msg::LandmarkArray msg;
  for (const auto & b : map.buoys) {
    asv_msgs::msg::Landmark l;
    l.id = b.id;
    l.name = b.name;
    l.colour.value = asv_sim::colour_code(b.colour);
    l.model = b.cls;
    l.radius = b.radius;
    l.x = b.x;
    l.y = b.y;
    msg.landmarks.push_back(l);
  }
  return msg;
}

}  // namespace

TEST(FakeDetection, SetupFromRepoFiles)
{
  const auto s = repo_setup();
  // asv_frames.yaml: lidar_link at x 0.25; zed_link 0.42, cam_wide_front_link 0.52.
  EXPECT_DOUBLE_EQ(s.params.lidar_x, 0.25);
  EXPECT_DOUBLE_EQ(s.params.lidar_y, 0.0);
  ASSERT_EQ(s.camera_names.size(), 2u);
  EXPECT_EQ(s.camera_names[0], "zed");
  EXPECT_EQ(s.camera_names[1], "cam_wide_front");
  EXPECT_DOUBLE_EQ(s.params.cameras[0].x, 0.42);
  EXPECT_DOUBLE_EQ(s.params.cameras[0].hfov, 1.2566);
  EXPECT_DOUBLE_EQ(s.params.cameras[1].x, 0.52);
  EXPECT_DOUBLE_EQ(s.params.cameras[1].hfov, 1.4835);
  // Lidar 10 Hz.
  EXPECT_EQ(s.scan_period_ns, 100'000'000);
  // asv_report.json limits (design.md section 5).
  EXPECT_DOUBLE_EQ(s.limits.at("A-0").lidar, 4.75);
  EXPECT_DOUBLE_EQ(s.limits.at("A-2").lidar, 10.75);
  EXPECT_DOUBLE_EQ(s.limits.at("Sur-Mark").lidar, 18.75);
  EXPECT_DOUBLE_EQ(s.limits.at("A-0").camera[0], 8.9);
  EXPECT_DOUBLE_EQ(s.limits.at("A-0").camera[1], 5.3);
  EXPECT_DOUBLE_EQ(s.limits.at("Sur-Mark").camera[0], 20.1);
  EXPECT_DOUBLE_EQ(s.limits.at("Sur-Mark").camera[1], 12.0);
  // Borrowed limits (class_limits_as).
  EXPECT_DOUBLE_EQ(s.limits.at("indicator").lidar, 18.75);
  EXPECT_DOUBLE_EQ(s.limits.at("other").camera[0], 20.1);
  // Nominal settings (decision M3.1 B).
  EXPECT_EQ(s.seed, 1u);
  EXPECT_DOUBLE_EQ(s.params.range_sigma, 0.10);
  EXPECT_DOUBLE_EQ(s.params.bearing_sigma, 0.02);
  EXPECT_DOUBLE_EQ(s.params.miss_probability, 0.0);
  EXPECT_DOUBLE_EQ(s.params.false_alarm_rate, 0.0);
  EXPECT_DOUBLE_EQ(s.params.colour_error_probability, 0.0);
  EXPECT_FALSE(s.params.centroid_bias);
}

TEST(FakeDetection, EveryMapClassHasLimits)
{
  const auto s = repo_setup();
  const auto objects = asv_sim::detector_objects(repo_map_message(), s.limits);
  ASSERT_EQ(objects.size(), 32u);
  int vessels = 0;
  for (const auto & o : objects) {
    vessels += o.vessel ? 1 : 0;
    EXPECT_EQ(o.camera_range.size(), 2u);
  }
  EXPECT_EQ(vessels, 5);
}

TEST(FakeDetection, UnknownClassThrows)
{
  asv_msgs::msg::LandmarkArray map;
  asv_msgs::msg::Landmark l;
  l.model = "dock";
  map.landmarks.push_back(l);
  EXPECT_THROW(asv_sim::detector_objects(map, repo_setup().limits), std::runtime_error);
}

TEST(FakeDetection, SpawnSeesTheTask1Gates)
{
  // At the spawn (0, 0, facing east) the four Task 1 gate Sur-Marks are within
  // 18.75 m of the lidar and inside the ZED's view and 20.1 m: all four with
  // their true colour. Noise off, so the numbers are exact.
  auto s = repo_setup();
  s.params.range_sigma = 0.0;
  s.params.bearing_sigma = 0.0;
  asv_core::FakeDetector det(
    s.params, asv_sim::detector_objects(repo_map_message(), s.limits), s.seed);
  const auto ds = det.scan(asv_core::Pose2{});
  ASSERT_EQ(ds.size(), 4u);
  // Sorted by bearing: gate 1 green (right), gate 2 green, gate 2 red, gate 1 red.
  EXPECT_EQ(*ds[0].true_id, 1u);
  EXPECT_EQ(*ds[3].true_id, 0u);
  EXPECT_NEAR(ds[3].range, std::hypot(5.0, 1.524), 1e-9);
  EXPECT_NEAR(ds[3].bearing, std::atan2(1.524, 5.0), 1e-9);
  EXPECT_EQ(ds[3].colour, asv_core::BuoyColour::red);
  EXPECT_EQ(ds[0].colour, asv_core::BuoyColour::green);
  for (const auto & d : ds) {
    EXPECT_FALSE(d.vessel);
  }
}

TEST(FakeDetection, MessageAndPose)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 3.0;
  odom.pose.pose.position.y = -2.0;
  odom.pose.pose.orientation.w = std::cos(0.25);  // yaw 0.5 rad
  odom.pose.pose.orientation.z = std::sin(0.25);
  const auto pose = asv_sim::pose_from_odometry(odom);
  EXPECT_DOUBLE_EQ(pose.x, 3.0);
  EXPECT_DOUBLE_EQ(pose.y, -2.0);
  EXPECT_NEAR(pose.yaw, 0.5, 1e-12);

  asv_core::Detection buoy;
  buoy.range = 4.0;
  buoy.bearing = -kPi / 4;
  buoy.colour = asv_core::BuoyColour::yellow;
  buoy.true_id = 12;
  asv_core::Detection vessel;
  vessel.range = 9.0;
  vessel.vessel = true;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 300'000'000;
  const auto msg = asv_sim::detection_message({buoy, vessel}, stamp);
  EXPECT_EQ(msg.header.frame_id, "base_link");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(asv_sim::stamp_nanoseconds(msg.header.stamp), 12'300'000'000LL);
  ASSERT_EQ(msg.detections.size(), 2u);
  EXPECT_DOUBLE_EQ(msg.detections[0].range, 4.0);
  EXPECT_DOUBLE_EQ(msg.detections[0].bearing, -kPi / 4);
  EXPECT_EQ(msg.detections[0].colour.value, asv_msgs::msg::BuoyColour::YELLOW);
  EXPECT_EQ(msg.detections[0].object_class, asv_msgs::msg::BuoyDetection::CLASS_BUOY);
  EXPECT_EQ(msg.detections[1].object_class, asv_msgs::msg::BuoyDetection::CLASS_OTHER);
  EXPECT_EQ(msg.detections[1].colour.value, asv_msgs::msg::BuoyColour::UNKNOWN);
}
