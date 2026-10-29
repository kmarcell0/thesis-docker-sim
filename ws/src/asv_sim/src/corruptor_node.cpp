// corruptor: turns the clean sensor topics into realistic ones (D1 a,
// design.md section 4). Only I/O; the maths is asv_core::ImuCorruptor and
// asv_sim::corrupt_imu() (imu_corruption.hpp). M2.6a: IMU only; GNSS follows
// in M2.6b/c.
//   in:  /asv/imu/data               clean IMU from Gazebo (M2.2)
//   out: /asv/sensors/imu/data       noisy gyro and accel, no orientation (D11)
//        /asv/ground_truth/imu_bias  the true bias in it, for the evaluation
// Run: ros2 run asv_sim corruptor --ros-args -p use_sim_time:=true
//        [-p noise_profile:=<name>]   (config/noise_profiles/<name>.yaml)
//
// asv.yaml and the profile are read from the package share directory, like
// the teleop node (T1 a), so ros2 run needs no parameter file.
#include <cstdint>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "yaml-cpp/yaml.h"

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

    const rclcpp::QoS qos(50);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/asv/sensors/imu/data", qos);
    bias_pub_ = create_publisher<sensor_msgs::msg::Imu>("/asv/ground_truth/imu_bias", qos);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/asv/imu/data", qos,
      [this](const sensor_msgs::msg::Imu & msg) {on_imu(msg);});
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

  std::unique_ptr<asv_core::ImuCorruptor> imu_;
  std::uint64_t received_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_, bias_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorruptorNode>());
  rclcpp::shutdown();
  return 0;
}
