#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_

#include <bitbots_splines/abstract_ik.h>
#include "dynup_utils.h"

namespace bitbots_dynup {
class DynupIK : public bitbots_splines::AbstractIK {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  bitbots_splines::JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
  bitbots_splines::JointGoals calculateDirectly(DynupResponse &ik_goals);
  void reset() override;
 private:
  robot_model::JointModelGroup *legs_joints_group_;
  robot_model::JointModelGroup *left_leg_joints_group_;
  robot_model::JointModelGroup *right_leg_joints_group_;
  robot_state::RobotStatePtr goal_state_;
};

}

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_
