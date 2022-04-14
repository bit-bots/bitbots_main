#include <bitbots_splines/pose_spline.h>
#include "bitbots_dynamic_kick/visualizer.h"

namespace bitbots_dynamic_kick {

Visualizer::Visualizer(const std::string &base_topic, rclcpp::Node::SharedPtr node) :
    node_(node),
    base_topic_(base_topic){
  /* make sure base_topic_ has consistent scheme */
  if (base_topic.compare(base_topic.size() - 1, 1, "/") != 0) {
    base_topic_ += "/";
  }

  /* create necessary publishers */
  goal_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(base_topic_ + "received_goal",1);
  foot_spline_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(base_topic_ + "flying_foot_spline",5);
  trunk_spline_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(base_topic_ + "trunk_spline",5);
  windup_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(base_topic_ + "kick_windup_point", 5);
  debug_publisher_ = node_->create_publisher<bitbots_dynamic_kick::msg::KickDebug>(base_topic_ + "debug",5);
}

void Visualizer::setParams(VisualizationParams params) {
  params_ = params;
}

void Visualizer::displayFlyingSplines(bitbots_splines::PoseSpline splines,
                                      const std::string &support_foot_frame) {
  if (foot_spline_publisher_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::MarkerArray path = getPath(splines, support_foot_frame, params_.spline_smoothness, node_);
  path.markers[0].color.g = 1;

  foot_spline_publisher_->publish(path);
}

void Visualizer::displayTrunkSplines(bitbots_splines::PoseSpline splines,
                                     const std::string &support_foot_frame) {
  if (trunk_spline_publisher_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::MarkerArray path = getPath(splines, support_foot_frame, params_.spline_smoothness, node_);
  path.markers[0].color.g = 1;

  trunk_spline_publisher_->publish(path);
}

void Visualizer::displayReceivedGoal(const bitbots_msgs::action::Kick::Goal &goal) {
  if (goal_publisher_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::Marker
      marker = getMarker({goal.ball_position.x, goal.ball_position.y, goal.ball_position.z}, goal.header.frame_id, node_);

  marker.ns = marker_ns_;
  marker.id = MarkerIDs::RECEIVED_GOAL;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.header.stamp = goal.header.stamp;
  marker.pose.orientation = goal.kick_direction;
  marker.scale.x = 0.08 + (goal.kick_speed / 3);
  marker.color.r = 1;

  goal_publisher_->publish(marker);
}

void Visualizer::displayWindupPoint(const Eigen::Vector3d &kick_windup_point, const std::string &support_foot_frame) {
  if (windup_publisher_->get_subscription_count() == 0)
    return;

  tf2::Vector3 tf_kick_windup_point;
  tf2::convert(kick_windup_point, tf_kick_windup_point);
  visualization_msgs::msg::Marker marker = getMarker(tf_kick_windup_point, support_foot_frame, node_);

  marker.ns = marker_ns_;
  marker.id = MarkerIDs::RECEIVED_GOAL;
  marker.color.g = 1;

  windup_publisher_->publish(marker);
}

void Visualizer::publishGoals(const KickPositions &positions,
                              const KickPositions &stabilized_positions,
                              const moveit::core::RobotStatePtr &robot_state,
                              KickPhase engine_phase) {
  /* only calculate the debug information if someone is subscribing */
  if (debug_publisher_->get_subscription_count() == 0) {
    return;
  }

  std::string support_foot_frame, flying_foot_frame;
  if (positions.is_left_kick) {
    support_foot_frame = "r_sole";
    flying_foot_frame = "l_sole";
  } else {
    support_foot_frame = "l_sole";
    flying_foot_frame = "r_sole";
  }

  /* Derive positions from robot state */
  Eigen::Isometry3d trunk_pose_ik_result = robot_state->getGlobalLinkTransform(support_foot_frame).inverse();
  Eigen::Isometry3d
      flying_foot_pose_ik_result = trunk_pose_ik_result * robot_state->getGlobalLinkTransform(flying_foot_frame);

  /* Calculate offsets */
  Eigen::Vector3d
      trunk_position_ik_offset = trunk_pose_ik_result.translation() - stabilized_positions.trunk_pose.translation();
  Eigen::Vector3d flying_foot_position_ik_offset =
      flying_foot_pose_ik_result.translation() - stabilized_positions.flying_foot_pose.translation();

  bitbots_dynamic_kick::msg::KickDebug msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.header.frame_id = support_foot_frame;
  msg.engine_phase = engine_phase;
  msg.engine_time = positions.engine_time;
  msg.trunk_pose_goal = tf2::toMsg(positions.trunk_pose);
  msg.trunk_pose_stabilized_goal = tf2::toMsg(stabilized_positions.trunk_pose);
  msg.trunk_pose_ik_result = tf2::toMsg(trunk_pose_ik_result);
  msg.trunk_position_ik_offset = tf2::toMsg(trunk_position_ik_offset);
  msg.flying_foot_pose_goal = tf2::toMsg(positions.flying_foot_pose);
  msg.flying_foot_pose_stabilized_goal = tf2::toMsg(stabilized_positions.flying_foot_pose);
  msg.flying_foot_pose_ik_result = tf2::toMsg(flying_foot_pose_ik_result);
  msg.flying_foot_position_ik_offset = tf2::toMsg(flying_foot_position_ik_offset);

  debug_publisher_->publish(msg);
}

}
