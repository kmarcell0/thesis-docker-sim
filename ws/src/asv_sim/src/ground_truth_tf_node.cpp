// ground_truth_tf: publishes the true pose of the boat as TF (M2.8). Only I/O;
// the conversion is asv_sim::transform_from_odometry() (ground_truth_tf.hpp).
//   in:  /asv/ground_truth/odom   nav_msgs/Odometry (50 Hz, frame world)
//   out: /tf                      world -> base_link, same stamp as the odometry
// base_link may have only one parent (design.md section 2): run this node OR
// slam_node (map -> base_link, M4.1), never both. sim.launch.py starts it
// unless ground_truth_tf:=false. Alone:
//      ros2 run asv_sim ground_truth_tf --ros-args -p use_sim_time:=true
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "asv_sim/ground_truth_tf.hpp"

class GroundTruthTf : public rclcpp::Node
{
public:
  GroundTruthTf()
  : Node("ground_truth_tf"), broadcaster_(*this)
  {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/asv/ground_truth/odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry & m) {
        broadcaster_.sendTransform(asv_sim::transform_from_odometry(m));
      });
  }

private:
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthTf>());
  rclcpp::shutdown();
  return 0;
}
