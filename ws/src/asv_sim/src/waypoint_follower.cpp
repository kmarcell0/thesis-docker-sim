// waypoint_follower: the Milestone 1 test driver as a ROS 2 node (M1.3).
// Only I/O: reads the parameters, keeps the latest odometry, and on a 20 Hz
// ROS-clock timer calls asv_sim::Follower (follower.hpp), which holds all the
// logic. Topics: milestone1_plan.md section 4; states: section 5.7.
// Run with use_sim_time:=true (the timer then runs in simulation time).
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "asv_core/angles.hpp"
#include "asv_sim/follower.hpp"

namespace
{

using Float64 = std_msgs::msg::Float64;

// Frame of the route, target and ground truth (docs/design.md section 2).
constexpr char kWorldFrame[] = "world";

}  // namespace

class WaypointFollower : public rclcpp::Node
{
public:
  WaypointFollower()
  : Node("waypoint_follower")
  {
    const auto config = load_config();
    follower_ = std::make_unique<asv_sim::Follower>(config);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      required<std::string>("odom_topic"), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_ = msg;});

    left_thrust_ = create_publisher<Float64>("/asv/thrusters/left/thrust", 10);
    right_thrust_ = create_publisher<Float64>("/asv/thrusters/right/thrust", 10);
    left_pos_ = create_publisher<Float64>("/asv/thrusters/left/pos", 10);
    right_pos_ = create_publisher<Float64>("/asv/thrusters/right/pos", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/asv/autonomy/target", 10);
    cte_pub_ = create_publisher<Float64>("/asv/autonomy/cross_track_error", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/asv/autonomy/status", 10);
    // Transient local: kept for subscribers that join later (RViz).
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/asv/autonomy/path", rclcpp::QoS(1).transient_local());
    publish_path(config.waypoints);

    // ROS clock, so the loop runs in simulation time (design.md section 3).
    const double rate = required<double>("control_rate");
    if (!(rate > 0.0)) {
      throw std::invalid_argument("control_rate must be > 0");
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {tick();});
  }

private:
  // A parameter that must be given (no default in code: parameters live in
  // YAML files). Declared with a type but no default, so rclcpp throws
  // UninitializedStaticallyTypedParameterException naming the parameter if
  // it is missing (checked in M1.3).
  template<typename T>
  T required(const std::string & name)
  {
    return declare_parameter<T>(name);
  }

  asv_sim::FollowerConfig load_config()
  {
    asv_sim::FollowerConfig c;
    const auto xs = required<std::vector<double>>("waypoints_x");
    const auto ys = required<std::vector<double>>("waypoints_y");
    if (xs.size() != ys.size()) {
      throw std::invalid_argument("waypoints_x and waypoints_y differ in length");
    }
    for (std::size_t i = 0; i < xs.size(); ++i) {
      c.waypoints.push_back({xs[i], ys[i]});
    }
    c.speeds = required<std::vector<double>>("speeds");
    c.switch_distance = required<double>("switch_distance");
    c.lookahead = required<double>("lookahead");
    c.heading.heading_kp = required<double>("heading_kp");
    c.heading.heading_kd = required<double>("heading_kd");
    c.heading.max_steer = required<double>("max_steer");
    c.heading.spin_threshold = required<double>("spin_threshold");
    c.heading.spin_kp = required<double>("spin_kp");
    c.heading.spin_max = required<double>("spin_max");
    c.heading.spin_small_kp = required<double>("spin_small_kp");
    c.course_min_speed = required<double>("course_min_speed");
    // Boat values from config/asv.yaml (hydrodynamics xU, xUU; thrusters).
    c.speed.drag_linear = required<double>("drag_linear");
    c.speed.drag_quadratic = required<double>("drag_quadratic");
    c.limits.max_thrust = required<double>("max_thrust");
    c.limits.min_thrust = required<double>("min_thrust");
    c.limits.steer_limit = required<double>("steer_limit");
    c.speed.max_thrust = c.limits.max_thrust;
    c.speed.speed_kp = required<double>("speed_kp");
    c.speed.speed_ki = required<double>("speed_ki");
    c.speed.stop_gain = required<double>("stop_gain");
    c.speed.turn_threshold = required<double>("turn_threshold");
    c.speed.min_throttle = required<double>("min_throttle");
    c.speed.max_throttle = required<double>("max_throttle");
    c.footprint.length = required<double>("boat_length");
    c.footprint.half_width = required<double>("boat_half_width");
    c.gate_margin = required<double>("gate_margin");
    c.gate_capture = required<double>("gate_capture");
    c.start_delay = required<double>("start_delay");
    c.max_run_time = required<double>("max_run_time");
    c.odom_timeout = required<double>("odom_timeout");
    load_buoys(c);
    return c;
  }

  // Buoys by name; gate buoys go into the gates, the rest into other_buoys.
  void load_buoys(asv_sim::FollowerConfig & c)
  {
    const auto names = required<std::vector<std::string>>("buoy_names");
    const auto coords = required<std::vector<double>>("buoy_coords");
    const auto radii = required<std::vector<double>>("buoy_radii");
    if (coords.size() != 2 * names.size() || radii.size() != names.size()) {
      throw std::invalid_argument("buoy_coords needs 2 and buoy_radii 1 value per buoy");
    }
    std::map<std::string, asv_sim::Buoy> buoys;
    for (std::size_t i = 0; i < names.size(); ++i) {
      buoys[names[i]] = {names[i], {coords[2 * i], coords[2 * i + 1]}, radii[i]};
    }
    const auto gates = required<std::vector<std::string>>("gate_names");
    const auto reds = required<std::vector<std::string>>("gate_red");
    const auto greens = required<std::vector<std::string>>("gate_green");
    if (reds.size() != gates.size() || greens.size() != gates.size()) {
      throw std::invalid_argument("gate_red and gate_green need one buoy per gate");
    }
    auto take = [&buoys](const std::string & name) {
        const auto it = buoys.find(name);
        if (it == buoys.end()) {
          throw std::invalid_argument("gate buoy '" + name + "' not in buoy_names");
        }
        const asv_sim::Buoy b = it->second;
        buoys.erase(it);
        return b;
      };
    for (std::size_t i = 0; i < gates.size(); ++i) {
      c.gates.push_back({gates[i], take(reds[i]), take(greens[i])});
    }
    for (const auto & name : names) {  // keep the file's order
      if (buoys.count(name)) {
        c.other_buoys.push_back(buoys.at(name));
      }
    }
  }

  void publish_path(const std::vector<asv_sim::Point2> & waypoints)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = kWorldFrame;
    for (const auto & w : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = w.x;
      pose.pose.position.y = w.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  // Odometry message -> NavState. Pose in world; twist in the body frame
  // (CLAUDE.md, ground truth): u = linear.x, v = linear.y, omega = angular.z.
  static asv_sim::NavState to_nav(const nav_msgs::msg::Odometry & m)
  {
    const auto & q = m.pose.pose.orientation;
    return asv_sim::NavState{
      rclcpp::Time(m.header.stamp).seconds(),
      {m.pose.pose.position.x, m.pose.pose.position.y},
      asv_core::yaw_from_quaternion(q.w, q.x, q.y, q.z),
      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z};
  }

  void tick()
  {
    const rclcpp::Time t = now();
    std::optional<asv_sim::NavState> nav;
    if (odom_) {
      nav = to_nav(*odom_);
    }
    const auto out = follower_->step(t.seconds(), nav);

    // Commands every cycle in every state: "stopped" is actively commanded.
    publish(left_thrust_, out.command.left_thrust);
    publish(right_thrust_, out.command.right_thrust);
    publish(left_pos_, out.command.left_pos);
    publish(right_pos_, out.command.right_pos);
    if (out.has_target) {
      geometry_msgs::msg::PointStamped target;
      target.header.stamp = t;
      target.header.frame_id = kWorldFrame;
      target.point.x = out.target.x;
      target.point.y = out.target.y;
      target_pub_->publish(target);
      publish(cte_pub_, out.cross_track);
    }
    for (const auto & e : out.events) {
      report(asv_sim::gate_event_text(e));
    }
    if (out.status != last_status_) {
      last_status_ = out.status;
      report(out.status);
      if (out.state == asv_sim::FollowerState::FINISHED ||
        out.state == asv_sim::FollowerState::STOPPED)
      {
        const auto closest = follower_->monitor().closest();
        RCLCPP_INFO(get_logger(), "closest buoy %s %.3f m, collision %s",
          closest.name.c_str(), closest.min_clearance,
          follower_->monitor().collision() ? "YES" : "no");
      }
    }
  }

  void report(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  static void publish(const rclcpp::Publisher<Float64>::SharedPtr & pub, double value)
  {
    Float64 msg;
    msg.data = value;
    pub->publish(msg);
  }

  std::unique_ptr<asv_sim::Follower> follower_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;  // latest message
  std::string last_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Float64>::SharedPtr left_thrust_, right_thrust_, left_pos_, right_pos_;
  rclcpp::Publisher<Float64>::SharedPtr cte_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
