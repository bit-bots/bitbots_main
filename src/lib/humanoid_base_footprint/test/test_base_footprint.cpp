// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "humanoid_base_footprint/base_footprint.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TestBaseFootprint : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestBaseFootprint, timestamp_matching)
{
  auto footprint_node = std::make_shared<humanoid_base_footprint::BaseFootprintBroadcaster>();
  auto node = std::make_shared<rclcpp::Node>("test_node");

  tf2_ros::TransformBroadcaster br(node);

  builtin_interfaces::msg::Time stamp(rclcpp::Time(1000000000, 000000001));

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  br.sendTransform(tf);

  tf.header.stamp = stamp;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "l_sole";
  br.sendTransform(tf);

  tf.header.stamp = stamp;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "r_sole";
  br.sendTransform(tf);

  tf2_ros::Buffer tfBuffer{node->get_clock()};
  tf2_ros::TransformListener tfListener{tfBuffer, node};

  rclcpp::sleep_for(std::chrono::milliseconds(50));  // Wait for timer callback to be called
  rclcpp::spin_some(footprint_node);
  rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for transforms to arrive into buffer

  ASSERT_NO_THROW(
  {
    auto tf = tfBuffer.lookupTransform("base_link", "base_footprint", stamp);
    EXPECT_EQ(tf.header.stamp, stamp);
  });
}

TEST_F(TestBaseFootprint, timestamp_non_matching)
{
  auto footprint_node = std::make_shared<humanoid_base_footprint::BaseFootprintBroadcaster>();
  auto node = std::make_shared<rclcpp::Node>("test_node");

  tf2_ros::TransformBroadcaster br(node);

  builtin_interfaces::msg::Time stamp(rclcpp::Time(1000000000, 000000001));

  geometry_msgs::msg::TransformStamped tf;

  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.header.stamp = rclcpp::Time(1000000000, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000001, 000000001);
  br.sendTransform(tf);

  tf.header.frame_id = "base_link";
  tf.child_frame_id = "l_sole";
  tf.header.stamp = rclcpp::Time(1000000000, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000002, 000000001);
  br.sendTransform(tf);

  tf.header.frame_id = "base_link";
  tf.child_frame_id = "r_sole";
  tf.header.stamp = rclcpp::Time(1000000000, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000003, 000000001);
  br.sendTransform(tf);

  tf2_ros::Buffer tfBuffer{node->get_clock()};
  tf2_ros::TransformListener tfListener{tfBuffer, node};

  rclcpp::sleep_for(std::chrono::milliseconds(50));  // Wait for timer callback to be called
  rclcpp::spin_some(footprint_node);
  rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for transforms to arrive into buffer

  ASSERT_NO_THROW(
  {
    auto tf = tfBuffer.lookupTransform("base_link", "base_footprint", tf2::TimePointZero);
    EXPECT_EQ(tf.header.stamp, rclcpp::Time(1000000001, 000000001));
  });
}

TEST_F(TestBaseFootprint, no_overlapping_time)
{
  auto footprint_node = std::make_shared<humanoid_base_footprint::BaseFootprintBroadcaster>();
  auto node = std::make_shared<rclcpp::Node>("test_node");

  tf2_ros::TransformBroadcaster br(node);

  builtin_interfaces::msg::Time stamp(rclcpp::Time(1000000000, 000000001));

  geometry_msgs::msg::TransformStamped tf;

  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.header.stamp = rclcpp::Time(1000000000, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000001, 000000001);
  br.sendTransform(tf);

  tf.header.frame_id = "base_link";
  tf.child_frame_id = "l_sole";
  tf.header.stamp = rclcpp::Time(1000000002, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000003, 000000001);
  br.sendTransform(tf);

  tf.header.frame_id = "base_link";
  tf.child_frame_id = "r_sole";
  tf.header.stamp = rclcpp::Time(1000000004, 000000001);
  br.sendTransform(tf);
  tf.header.stamp = rclcpp::Time(1000000005, 000000001);
  br.sendTransform(tf);

  tf2_ros::Buffer tfBuffer{node->get_clock()};
  tf2_ros::TransformListener tfListener{tfBuffer, node};

  rclcpp::sleep_for(std::chrono::milliseconds(50));  // Wait for timer callback to be called
  rclcpp::spin_some(footprint_node);
  rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for transforms to arrive into buffer

  ASSERT_THROW(
  {
    tfBuffer.lookupTransform("base_link", "base_footprint", tf2::TimePointZero);
  }, tf2::TransformException);
}

TEST_F(TestBaseFootprint, test_tf_listener_node_isnt_created)
{
  // Ensure the tf listener does not create its own tf listener node.
  // See https://github.com/ros-sports/humanoid_base_footprint/pull/41 for more details.
  auto footprint_node = std::make_shared<humanoid_base_footprint::BaseFootprintBroadcaster>();
  auto nodes = footprint_node->get_node_names();
  ASSERT_EQ(nodes.size(), 1u);
}
