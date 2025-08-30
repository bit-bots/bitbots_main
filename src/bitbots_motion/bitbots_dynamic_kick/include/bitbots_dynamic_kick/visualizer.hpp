//
// Created by ftsell on 6/19/19.
//

#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_

#include <bitbots_dynamic_kick/kick_utils.hpp>
#include <bitbots_dynamic_kick/msg/kick_debug.hpp>
#include <bitbots_msgs/action/kick.hpp>
#include <bitbots_splines/abstract_visualizer.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <bitbots_splines/spline_container.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace bitbots_dynamic_kick {

enum MarkerIDs {
  RECEIVED_GOAL = 1,
  KICK_WINDUP_POINT = 2,
  KICK_STABILIZING_POINT = 3,
};

struct VisualizationParams {
  int spline_smoothness;
};

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:
  explicit Visualizer(const std::string &base_topic, rclcpp::Node::SharedPtr node);

  void setParams(VisualizationParams params);

  void displayReceivedGoal(const bitbots_msgs::action::Kick::Goal &goal);

  void displayFlyingSplines(bitbots_splines::PoseSpline splines, const std::string &support_foot_frame);

  void displayTrunkSplines(bitbots_splines::PoseSpline splines, const std::string &support_foot_frame);

  void displayWindupPoint(const Eigen::Vector3d &kick_windup_point, const std::string &support_foot_frame);

  void publishGoals(const KickPositions &positions, const KickPositions &stabilized_positions,
                    const moveit::core::RobotStatePtr &robot_state, KickPhase engine_phase);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foot_spline_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trunk_spline_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr windup_publisher_;
  rclcpp::Publisher<bitbots_dynamic_kick::msg::KickDebug>::SharedPtr debug_publisher_;
  std::string base_topic_;
  const std::string marker_ns_ = "bitbots_dynamic_kick";
  VisualizationParams params_;
};
}  // namespace bitbots_dynamic_kick

#endif  // BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_
