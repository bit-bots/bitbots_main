#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#include <bitbots_quintic_walk/bitbots_quintic_walk_parameters.hpp>
#include <bitbots_quintic_walk/walk_utils.hpp>
#include <bitbots_quintic_walk/ik.hpp>
#include <bitbots_splines/abstract_ik.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.hpp>

namespace bitbots_quintic_walk {

class WalkIK : public bitbots_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp::Node::SharedPtr node, walking::Params::Node::Ik config);

  bitbots_splines::JointGoals calculate(const WalkResponse& ik_goals);
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setConfig(walking::Params::Node::Ik config);

  const std::vector<std::string> getLeftLegJointNames();
  const std::vector<std::string> getRightLegJointNames();

  const geometry_msgs::msg::Pose get_right_goal();
  const geometry_msgs::msg::Pose get_left_goal();

 private:
  rclcpp::Node::SharedPtr node_;

  geometry_msgs::msg::Pose right_foot_goal_;
  geometry_msgs::msg::Pose left_foot_goal_;

  walking::Params::Node::Ik config_;
};
}  // namespace bitbots_quintic_walk
#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
