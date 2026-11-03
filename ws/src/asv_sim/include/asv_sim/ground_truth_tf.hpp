// Ground-truth TF glue (M2.8): turns the ground-truth odometry into the TF
// edge world -> base_link, so RViz and later nodes can place base_link and
// every sensor frame in the world. Message glue only (no node), so it is unit
// tested; the node ground_truth_tf does the I/O.
#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace asv_sim
{

// The odometry pose is T_WB, the pose of base_link (B) in world (W): position
// p and orientation q. The transform carries exactly that: translation = p,
// rotation = q. Stamp and frame names are copied from the message
// (header.frame_id -> header.frame_id, child_frame_id -> child_frame_id).
// The twist is not used: TF holds poses only.
geometry_msgs::msg::TransformStamped transform_from_odometry(
  const nav_msgs::msg::Odometry & odom);

}  // namespace asv_sim
