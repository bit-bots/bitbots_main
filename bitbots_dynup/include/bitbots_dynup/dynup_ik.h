#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_

#include <bitbots_splines/abstract_ik.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dynup_utils.h"

namespace bitbots_dynup {
class DynupIK : public bitbots_splines::AbstractIK<DynupResponse> {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  bitbots_splines::JointGoals calculate(const DynupResponse &ik_goals) override;
  void reset() override;
 private:
  moveit::core::JointModelGroupPtr all_joints_group_;
  moveit::core::JointModelGroupPtr arm_joints_group_;
  moveit::core::JointModelGroupPtr leg_joints_group_;
  robot_state::RobotStatePtr goal_state_;
};

}

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_
