// corruptor: turns the clean sensor topics into realistic ones (D1 a,
// design.md section 4). Only I/O; the maths is asv_core::ImuCorruptor and
// asv_core::GnssCorruptor, the message glue asv_sim::corrupt_imu()
// (imu_corruption.hpp) and asv_sim::GnssCorruption (gnss_corruption.hpp).
// M2.6a IMU, M2.6b GNSS position, M2.6c GNSS velocity and heading.
//   in:  /asv/imu/data                  clean IMU from Gazebo (M2.2)
//        /asv/gnss/fix                  clean primary antenna fix (M2.2)
//        /asv/ground_truth/odom         true yaw, for the heading (M2.6c)
//   out: /asv/sensors/imu/data          noisy gyro and accel, no orientation (D11)
//        /asv/sensors/gnss/fix          fix with the error of the current mode;
//                                       nothing in an outage
//        /asv/sensors/gnss/velocity     antenna velocity (ENU) + noise, one
//                                       epoch late; nothing in an outage
//        /asv/sensors/gnss/heading      azimuth of the bow + noise; nothing in
//                                       an outage
//        /asv/ground_truth/imu_bias     the true IMU bias, for the evaluation
//        /asv/ground_truth/gnss_status  the true receiver mode, every epoch
//        /asv/ground_truth/gnss_error   the true slow GNSS error (ENU, m)
// Started by sim.launch.py (M2.6d; launch arguments corruptor:=false,
// noise_profile:=<name>). Alone:
//      ros2 run asv_sim corruptor --ros-args -p use_sim_time:=true
//        [-p noise_profile:=<name>]   (config/noise_profiles/<name>.yaml)
//
// asv.yaml and the profile are read from the package share directory, like
// the teleop node (T1 a), so ros2 run needs no parameter file.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "yaml-cpp/yaml.h"

#include "asv_sim/gnss_corruption.hpp"
#include "asv_sim/imu_corruption.hpp"

class CorruptorNode : public rclcpp::Node
{
public:
  CorruptorNode()
  : Node("corruptor")
  {
    const std::string profile_name = declare_parameter<std::string>("noise_profile", "nominal");
    const std::string config =
      ament_index_cpp::get_package_share_directory("asv_sim") + "/config";
    const YAML::Node asv = YAML::LoadFile(config + "/asv.yaml");
    const YAML::Node profile =
      YAML::LoadFile(config + "/noise_profiles/" + profile_name + ".yaml");

    const auto p = asv_sim::imu_error_params(asv, profile);
    const std::uint64_t seed = asv_sim::profile_seed(profile);
    imu_ = std::make_unique<asv_core::ImuCorruptor>(p, seed);
    RCLCPP_INFO(
      get_logger(),
      "profile '%s', seed %lu; gyro eta %.4g, b0 %.4g, b_ss %.4g; accel eta %.4g, "
      "b0 %.4g, b_ss %.4g; T_c %.4g s",
      profile_name.c_str(), static_cast<unsigned long>(seed), p.gyro.density,
      p.gyro.turn_on_sigma, p.gyro.drift_sigma, p.accel.density, p.accel.turn_on_sigma,
      p.accel.drift_sigma, p.gyro.drift_time);

    const auto g = asv_sim::gnss_error_params(asv, profile);
    auto schedule = asv_sim::gnss_schedule(profile);
    std::string modes;
    for (const auto & e : schedule.entries()) {
      modes += " " + asv_core::gnss_mode_name(e.mode) + "@" + std::to_string(e.from) + "s";
    }
    gnss_ = std::make_unique<asv_sim::GnssCorruption>(g, std::move(schedule), seed);
    RCLCPP_INFO(
      get_logger(),
      "GNSS rtk_fixed white %.4g, slow %.4g m; velocity %.4g m/s; heading %.4g rad; "
      "schedule:%s",
      g.rtk_fixed.white_h, g.rtk_fixed.slow_h, g.velocity_white,
      g.heading_white.value_or(-1.0), modes.c_str());

    const rclcpp::QoS qos(50);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/asv/sensors/imu/data", qos);
    bias_pub_ = create_publisher<sensor_msgs::msg::Imu>("/asv/ground_truth/imu_bias", qos);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/asv/imu/data", qos,
      [this](const sensor_msgs::msg::Imu & msg) {on_imu(msg);});

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/asv/sensors/gnss/fix", qos);
    status_pub_ =
      create_publisher<asv_msgs::msg::GnssStatus>("/asv/ground_truth/gnss_status", qos);
    gnss_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/asv/ground_truth/gnss_error", qos);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/asv/sensors/gnss/velocity", qos);
    heading_pub_ =
      create_publisher<asv_msgs::msg::GnssHeading>("/asv/sensors/gnss/heading", qos);
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/asv/gnss/fix", qos,
      [this](const sensor_msgs::msg::NavSatFix & msg) {on_fix(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/asv/ground_truth/odom", qos,
      [this](const nav_msgs::msg::Odometry & msg) {publish_headings(gnss_->on_odom(msg));});
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & clean)
  {
    const auto out = asv_sim::corrupt_imu(clean, *imu_);
    ++received_;
    if (!out) {
      if (received_ > 1) {
        // Stamp not later than the previous one (e.g. the simulation was
        // reset): dropped, it would give dt <= 0.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "IMU message with an old stamp dropped");
      }
      return;
    }
    imu_pub_->publish(out->data);
    bias_pub_->publish(out->bias);
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & clean)
  {
    const auto out = gnss_->on_fix(clean);
    if (!out.position) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "GNSS fix with an old stamp dropped");
      return;
    }
    const auto & pos = *out.position;
    status_pub_->publish(pos.status);
    if (pos.fix) {
      fix_pub_->publish(*pos.fix);
      gnss_error_pub_->publish(*pos.slow_error);
    }
    if (out.velocity) {
      velocity_pub_->publish(*out.velocity);
    }
    publish_headings(out.headings);
  }

  void publish_headings(const std::vector<asv_msgs::msg::GnssHeading> & headings)
  {
    for (const auto & h : headings) {
      heading_pub_->publish(h);
    }
    if (gnss_->dropped_headings() > dropped_reported_) {
      dropped_reported_ = gnss_->dropped_headings();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%zu GNSS headings dropped: no ground-truth yaw for their stamp", dropped_reported_);
    }
  }

  std::unique_ptr<asv_core::ImuCorruptor> imu_;
  std::unique_ptr<asv_sim::GnssCorruption> gnss_;
  std::size_t dropped_reported_ = 0;
  std::uint64_t received_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_, bias_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<asv_msgs::msg::GnssStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gnss_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<asv_msgs::msg::GnssHeading>::SharedPtr heading_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorruptorNode>());
  rclcpp::shutdown();
  return 0;
}
