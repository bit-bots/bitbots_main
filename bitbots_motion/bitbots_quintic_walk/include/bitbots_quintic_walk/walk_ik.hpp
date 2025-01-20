#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#include <bitbots_quintic_walk/walk_utils.hpp>
#include <bitbots_quintic_walk_parameters.hpp>
#include <bitbots_splines/abstract_ik.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace bitbots_quintic_walk {

class WalkIK : public bitbots_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp::Node::SharedPtr node, walking::Params::Node::Ik config);

  bitbots_splines::JointGoals calculate(const WalkResponse& ik_goals);
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setConfig(walking::Params::Node::Ik config);

  const std::vector<std::string>& getLeftLegJointNames();
  const std::vector<std::string>& getRightLegJointNames();

  moveit::core::RobotStatePtr get_goal_state();

 private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup* legs_joints_group_;
  const moveit::core::JointModelGroup* left_leg_joints_group_;
  const moveit::core::JointModelGroup* right_leg_joints_group_;
  walking::Params::Node::Ik config_;
};
}  // namespace bitbots_quintic_walk
#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
