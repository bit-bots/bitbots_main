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

  void publishArrowMarker(std::string name_space, std::string frame, geometry_msgs::msg::Pose pose, float r, float g,
                          float b, float a);

  void publishEngineDebug(WalkResponse response);
  void publishIKDebug(WalkResponse response, moveit::core::RobotStatePtr current_state,
                      bitbots_splines::JointGoals joint_goals);
  void publishWalkMarkers(WalkResponse response);

  void init(moveit::core::RobotModelPtr kinematic_model);

 private:
  rclcpp::Node::SharedPtr node_;

  walking::Params::Node::Tf tf_config_;

  int marker_id_ = 1;

  rclcpp::Publisher<bitbots_quintic_walk::msg::WalkDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<bitbots_quintic_walk::msg::WalkEngineDebug>::SharedPtr pub_engine_debug_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_debug_marker_;

  moveit::core::RobotModelPtr kinematic_model_;
};
}  // namespace bitbots_quintic_walk

#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
