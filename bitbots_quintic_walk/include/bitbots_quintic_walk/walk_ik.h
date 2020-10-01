#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#include "bitbots_quintic_walk/walk_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/robot_state.h>
#include "bitbots_splines/reference_goals.h"
#include "bitbots_splines/abstract_ik.h"
namespace bitbots_quintic_walk {

class WalkIK : public bitbots_splines::AbstractIK<WalkResponse> {
 public:
  WalkIK();

  bitbots_splines::JointGoals calculate(const WalkResponse &ik_goals) override;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setIKTimeout(double timeout);

  const std::vector<std::string>& getLeftLegJointNames();
  const std::vector<std::string>& getRightLegJointNames();

  robot_state::RobotStatePtr get_goal_state();

 private:
  robot_state::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup *legs_joints_group_;
  const moveit::core::JointModelGroup *left_leg_joints_group_;
  const moveit::core::JointModelGroup *right_leg_joints_group_;

  double ik_timeout_;

};
} // namespace bitbots_quintic_walk
#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
