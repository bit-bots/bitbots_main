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

#ifndef HUMANOID_BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_
#define HUMANOID_BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>
#include <biped_interfaces/msg/phase.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// taken from REP120
//
// base_footprint
//
// The base_footprint is the representation of the robot position on the floor.
// The floor is usually the level where the supporting leg rests,
// i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left and right sole height
// respecitvely.
// The translation component of the frame should be the barycenter of the feet projections on the
// floor.
// With respect to the odom frame, the roll and pitch angles should be zero and the yaw angle should
// correspond to the base_link yaw angle.
//
// Rationale: base_footprint provides a fairly stable 2D planar representation of the humanoid even
// while walking and swaying with the base_link.

namespace humanoid_base_footprint
{

class BaseFootprintBroadcaster : public rclcpp::Node
{
public:
  explicit BaseFootprintBroadcaster(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  void loop();

private:
  geometry_msgs::msg::TransformStamped tf_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  bool is_left_support_, got_support_foot_;
  std::string base_link_frame_, base_footprint_frame_, r_sole_frame_, l_sole_frame_, odom_frame_;
  void supportFootCallback(const biped_interfaces::msg::Phase msg);
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr walking_support_foot_subscriber_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr
    dynamic_kick_support_foot_subscriber_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();
};

}  // namespace humanoid_base_footprint

#endif  // HUMANOID_BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_
