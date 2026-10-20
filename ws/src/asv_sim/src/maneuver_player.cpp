// maneuver_player: plays a timed (throttle, steer, spin) table open loop
// (M1.7a). Only I/O: reads the parameters, and on a 20 Hz ROS-clock timer
// calls asv_sim::ManeuverPlayer (maneuver.hpp), which holds all the logic.
// Reads no pose: nothing is corrected. Never run it together with the
// waypoint follower (both publish the thruster commands).
// Run with use_sim_time:=true (the timer then runs in simulation time).
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "asv_sim/maneuver.hpp"

namespace
{
using Float64 = std_msgs::msg::Float64;
}  // namespace

class ManeuverPlayerNode : public rclcpp::Node
{
public:
  ManeuverPlayerNode()
  : Node("maneuver_player")
  {
    const auto maneuver = asv_sim::Maneuver::from_columns(
      required<std::vector<double>>("times"), required<std::vector<double>>("throttle"),
      required<std::vector<double>>("steer"), required<std::vector<double>>("spin"));
    // Boat values from config/asv.yaml (thrusters), passed by the launch file.
    const asv_sim::ThrusterLimits limits{
      required<double>("max_thrust"), required<double>("min_thrust"),
      required<double>("steer_limit")};
    player_ = std::make_unique<asv_sim::ManeuverPlayer>(
      maneuver, required<double>("start_delay"), limits);
    RCLCPP_INFO(get_logger(), "maneuver: %zu rows, %.1f s",
      maneuver.command_rows(), maneuver.end_time());

    left_thrust_ = create_publisher<Float64>("/asv/thrusters/left/thrust", 10);
    right_thrust_ = create_publisher<Float64>("/asv/thrusters/right/thrust", 10);
    left_pos_ = create_publisher<Float64>("/asv/thrusters/left/pos", 10);
    right_pos_ = create_publisher<Float64>("/asv/thrusters/right/pos", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/asv/maneuver/status", 10);

    // ROS clock, so the table runs in simulation time (design.md section 3).
    const double rate = required<double>("control_rate");
    if (!(rate > 0.0)) {
      throw std::invalid_argument("control_rate must be > 0");
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {tick();});
  }

private:
  // A parameter that must be given (no default in code), as in
  // waypoint_follower.cpp.
  template<typename T>
  T required(const std::string & name)
  {
    return declare_parameter<T>(name);
  }

  void tick()
  {
    const bool was_started = player_->started();
    const auto out = player_->step(now().seconds());
    if (!was_started && player_->started()) {
      // M1.7b lines up runs on this time.
      RCLCPP_INFO(get_logger(), "start at t = %.3f s (simulation time)",
        player_->start_time());
    }
    // Commands every cycle in every state: zero is actively commanded (P3).
    publish(left_thrust_, out.command.left_thrust);
    publish(right_thrust_, out.command.right_thrust);
    publish(left_pos_, out.command.left_pos);
    publish(right_pos_, out.command.right_pos);
    if (out.status != last_status_) {
      last_status_ = out.status;
      std_msgs::msg::String msg;
      msg.data = out.status;
      status_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "%s", out.status.c_str());
    }
  }

  static void publish(const rclcpp::Publisher<Float64>::SharedPtr & pub, double value)
  {
    Float64 msg;
    msg.data = value;
    pub->publish(msg);
  }

  std::unique_ptr<asv_sim::ManeuverPlayer> player_;
  std::string last_status_;
  rclcpp::Publisher<Float64>::SharedPtr left_thrust_, right_thrust_, left_pos_, right_pos_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManeuverPlayerNode>());
  rclcpp::shutdown();
  return 0;
}
