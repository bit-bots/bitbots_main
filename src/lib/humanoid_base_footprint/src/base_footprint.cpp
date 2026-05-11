// Copyright 2022 Hamburg Bit-Bots
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

#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <functional>
#include <humanoid_base_footprint/base_footprint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// This has to be included after #include <tf2_geometry_msgs/tf2_geometyr_msgs.hpp>, due to reasons
// explained in https://github.com/ros2/geometry2/pull/485
// "tf2/utils.h" is used instead of <tf2/utils.h> to prevent ament_cpplint from complaining about
// including a C system header after C++ system headers.
#include "tf2/utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace humanoid_base_footprint
{

BaseFootprintBroadcaster::BaseFootprintBroadcaster(const rclcpp::NodeOptions &)
: Node("base_footprint"),
  tfBuffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
  tfListener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this))
{
  // setup tf listener and broadcaster as class members
  base_link_frame_ = this->declare_parameter<std::string>("base_link_frame", "base_link");
  base_footprint_frame_ = this->declare_parameter<std::string>(
    "base_footprint_frame", "base_footprint");
  r_sole_frame_ = this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  l_sole_frame_ = this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

  got_support_foot_ = false;

  this->declare_parameter<std::vector<std::string>>("support_state_topics", {"walk_support_state"});
  std::vector<std::string> support_state_topics;
  this->get_parameter("support_state_topics", support_state_topics);
  for (auto topic : support_state_topics) {
    this->create_subscription<biped_interfaces::msg::Phase>(
      topic,
      1,
      std::bind(
        &BaseFootprintBroadcaster::supportFootCallback,
        this,
        _1));
  }

  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  double publish_frequency = this->declare_parameter<double>("publish_frequency", 30.0);
  std::chrono::milliseconds publish_interval_ms =
    std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_frequency));
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(publish_interval_ms),
    std::bind(&BaseFootprintBroadcaster::timerCallback, this));
}

void BaseFootprintBroadcaster::timerCallback()
{
  try {
    geometry_msgs::msg::TransformStamped tf_right =
      tfBuffer_->lookupTransform(base_link_frame_, r_sole_frame_, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped tf_left =
      tfBuffer_->lookupTransform(base_link_frame_, l_sole_frame_, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped odom =
      tfBuffer_->lookupTransform(base_link_frame_, odom_frame_, tf2::TimePointZero);

    auto stamp = std::min(
      {
        rclcpp::Time{tf_right.header.stamp},
        rclcpp::Time{tf_left.header.stamp},
        rclcpp::Time{odom.header.stamp}
      });
    tf_right = tfBuffer_->lookupTransform(base_link_frame_, r_sole_frame_, stamp);
    tf_left = tfBuffer_->lookupTransform(base_link_frame_, l_sole_frame_, stamp);
    odom = tfBuffer_->lookupTransform(base_link_frame_, odom_frame_, stamp);

    geometry_msgs::msg::TransformStamped support_foot, non_support_foot;

    if (got_support_foot_) {
      if (is_left_support_) {
        support_foot = tf_left;
        non_support_foot = tf_right;
      } else {
        support_foot = tf_right;
        non_support_foot = tf_left;
      }
    } else {
      // check which foot is support foot (which foot is on the ground)
      if (tf_right.transform.translation.z < tf_left.transform.translation.z) {
        support_foot = tf_right;
        non_support_foot = tf_left;
      } else {
        support_foot = tf_left;
        non_support_foot = tf_right;
      }
    }
    // get the position of the non-support foot in the support frame, used for computing the
    // barycenter
    geometry_msgs::msg::TransformStamped non_support_foot_in_support_foot_frame =
      tfBuffer_->lookupTransform(
      support_foot.child_frame_id, non_support_foot.child_frame_id,
      stamp);

    geometry_msgs::msg::TransformStamped
      support_to_base_link = tfBuffer_->lookupTransform(
      support_foot.header.frame_id,
      support_foot.child_frame_id,
      stamp);

    geometry_msgs::msg::PoseStamped base_footprint;
    // z at ground leven (support foot height)
    base_footprint.pose.position.z = 0;
    // x and y at barycenter of feet projections on the ground
    base_footprint.pose.position.x =
      non_support_foot_in_support_foot_frame.transform.translation.x / 2;
    base_footprint.pose.position.y =
      non_support_foot_in_support_foot_frame.transform.translation.y / 2;

    double yaw = rot_conv::FYawOfQuat(
      Eigen::Quaterniond(
        odom.transform.rotation.w,
        odom.transform.rotation.x,
        odom.transform.rotation.y,
        odom.transform.rotation.z));

    // pitch and roll from support foot, yaw from base link
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);

    tf2::Quaternion odom_rot;
    tf2::fromMsg(odom.transform.rotation, odom_rot);

    base_footprint.pose.orientation = tf2::toMsg(odom_rot * rotation.inverse());

    // transform the position and orientation of the base footprint into the base_link frame
    geometry_msgs::msg::PoseStamped base_footprint_in_base_link;
    tf2::doTransform(base_footprint, base_footprint_in_base_link, support_to_base_link);

    // set the broadcasted transform to the position and orientation of the base footprint
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = base_link_frame_;
    tf.child_frame_id = base_footprint_frame_;
    tf.transform.translation.x = base_footprint_in_base_link.pose.position.x;
    tf.transform.translation.y = base_footprint_in_base_link.pose.position.y;
    tf.transform.translation.z = base_footprint_in_base_link.pose.position.z;
    tf.transform.rotation = base_footprint.pose.orientation;
    tfBroadcaster_->sendTransform(tf);
  } catch (const tf2::TransformException & ex) {
    // RCLCPP_WARN(this->get_logger(), ex.what());
  }
}

void BaseFootprintBroadcaster::supportFootCallback(const biped_interfaces::msg::Phase msg)
{
  got_support_foot_ = true;
  is_left_support_ = (msg.phase == biped_interfaces::msg::Phase::LEFT_STANCE);
}

}  // namespace humanoid_base_footprint

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(humanoid_base_footprint::BaseFootprintBroadcaster)
