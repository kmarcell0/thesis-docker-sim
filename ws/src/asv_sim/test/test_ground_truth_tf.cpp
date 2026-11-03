// Tests of the ground-truth TF glue (M2.8): the odometry pose becomes the
// transform world -> base_link, with the stamp and frame names copied.
#include <gtest/gtest.h>

#include "asv_sim/ground_truth_tf.hpp"

namespace
{

nav_msgs::msg::Odometry sample_odom()
{
  nav_msgs::msg::Odometry m;
  m.header.stamp.sec = 12;
  m.header.stamp.nanosec = 340000000;
  m.header.frame_id = "world";
  m.child_frame_id = "base_link";
  m.pose.pose.position.x = 1.5;
  m.pose.pose.position.y = -2.25;
  m.pose.pose.position.z = 0.125;
  // a unit quaternion with all four parts different (yaw 0.6, small roll/pitch)
  m.pose.pose.orientation.x = 0.0123;
  m.pose.pose.orientation.y = -0.0234;
  m.pose.pose.orientation.z = 0.2951;
  m.pose.pose.orientation.w = 0.9551;
  m.twist.twist.linear.x = 2.0;
  m.twist.twist.angular.z = 0.3;
  return m;
}

}  // namespace

TEST(GroundTruthTf, CopiesStampAndFrames)
{
  const auto t = asv_sim::transform_from_odometry(sample_odom());
  EXPECT_EQ(t.header.stamp.sec, 12);
  EXPECT_EQ(t.header.stamp.nanosec, 340000000u);
  EXPECT_EQ(t.header.frame_id, "world");
  EXPECT_EQ(t.child_frame_id, "base_link");
}

TEST(GroundTruthTf, PoseBecomesTransform)
{
  const auto o = sample_odom();
  const auto t = asv_sim::transform_from_odometry(o);
  // Exact copies: no arithmetic between the two messages.
  EXPECT_EQ(t.transform.translation.x, o.pose.pose.position.x);
  EXPECT_EQ(t.transform.translation.y, o.pose.pose.position.y);
  EXPECT_EQ(t.transform.translation.z, o.pose.pose.position.z);
  EXPECT_EQ(t.transform.rotation.x, o.pose.pose.orientation.x);
  EXPECT_EQ(t.transform.rotation.y, o.pose.pose.orientation.y);
  EXPECT_EQ(t.transform.rotation.z, o.pose.pose.orientation.z);
  EXPECT_EQ(t.transform.rotation.w, o.pose.pose.orientation.w);
}

TEST(GroundTruthTf, TwistIsNotUsed)
{
  auto a = sample_odom();
  auto b = sample_odom();
  b.twist.twist.linear.x = -7.0;
  b.twist.twist.angular.z = 1.0;
  EXPECT_EQ(asv_sim::transform_from_odometry(a), asv_sim::transform_from_odometry(b));
}

TEST(GroundTruthTf, OtherFrameNamesPassThrough)
{
  // slam_node (M4.1) uses map -> base_link; the glue must not hard-code world.
  auto o = sample_odom();
  o.header.frame_id = "map";
  const auto t = asv_sim::transform_from_odometry(o);
  EXPECT_EQ(t.header.frame_id, "map");
  EXPECT_EQ(t.child_frame_id, "base_link");
}
