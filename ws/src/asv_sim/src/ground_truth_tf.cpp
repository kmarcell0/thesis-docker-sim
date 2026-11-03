#include "asv_sim/ground_truth_tf.hpp"

namespace asv_sim
{

geometry_msgs::msg::TransformStamped transform_from_odometry(
  const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::TransformStamped t;
  t.header = odom.header;
  t.child_frame_id = odom.child_frame_id;
  t.transform.translation.x = odom.pose.pose.position.x;
  t.transform.translation.y = odom.pose.pose.position.y;
  t.transform.translation.z = odom.pose.pose.position.z;
  t.transform.rotation = odom.pose.pose.orientation;
  return t;
}

}  // namespace asv_sim
