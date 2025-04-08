#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_

#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <bitbots_quintic_walk/walk_engine.hpp>
#include <bitbots_quintic_walk/walk_utils.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bitbots_quintic_walk/msg/walk_debug.hpp"
#include "bitbots_quintic_walk/msg/walk_engine_debug.hpp"
#include "bitbots_splines/abstract_ik.hpp"
#include "bitbots_splines/abstract_visualizer.hpp"

namespace bitbots_quintic_walk {
class WalkVisualizer : public bitbots_splines::AbstractVisualizer {
 public:
  explicit WalkVisualizer(rclcpp::Node::SharedPtr node, walking::Params::Node::Tf tf_config);

  visualization_msgs::msg::Marker createArrowMarker(const std::string &name_space, const std::string &frame,
                                                    const geometry_msgs::msg::Pose &pose,
                                                    const std_msgs::msg::ColorRGBA &color);

  std::pair<bitbots_quintic_walk::msg::WalkEngineDebug, visualization_msgs::msg::MarkerArray> buildEngineDebugMsgs(
      WalkResponse response);
  std::pair<bitbots_quintic_walk::msg::WalkDebug, visualization_msgs::msg::MarkerArray> buildIKDebugMsgs(
      WalkResponse response, moveit::core::RobotStatePtr current_state, bitbots_splines::JointGoals joint_goals);
  visualization_msgs::msg::MarkerArray buildWalkMarkers(WalkResponse response);

  void init(moveit::core::RobotModelPtr kinematic_model);

  void publishDebug(const WalkResponse &current_response, const moveit::core::RobotStatePtr &current_state,
                    const bitbots_splines::JointGoals &motor_goals);

  std_msgs::msg::ColorRGBA colorFactory(double r, double g, double b) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    return color;
  }

  const std_msgs::msg::ColorRGBA BLACK = colorFactory(0.0, 0.0, 0.0);
  const std_msgs::msg::ColorRGBA BLUE = colorFactory(0.0, 0.0, 1.0);
  const std_msgs::msg::ColorRGBA GREEN = colorFactory(0.0, 1.0, 0.0);
  const std_msgs::msg::ColorRGBA ORANGE = colorFactory(1.0, 0.5, 0.0);
  const std_msgs::msg::ColorRGBA RED = colorFactory(1.0, 0.0, 0.0);
  const std_msgs::msg::ColorRGBA WHITE = colorFactory(1.0, 1.0, 1.0);
  const std_msgs::msg::ColorRGBA YELLOW = colorFactory(1.0, 1.0, 0.0);

 private:
  rclcpp::Node::SharedPtr node_;

  walking::Params::Node::Tf tf_config_;

  rclcpp::Publisher<bitbots_quintic_walk::msg::WalkDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<bitbots_quintic_walk::msg::WalkEngineDebug>::SharedPtr pub_engine_debug_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  moveit::core::RobotModelPtr kinematic_model_;
};
}  // namespace bitbots_quintic_walk

#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
