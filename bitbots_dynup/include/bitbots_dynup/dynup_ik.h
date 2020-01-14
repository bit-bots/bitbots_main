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
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
 private:
  moveit::core::JointModelGroupPtr all_joints_group_;
  moveit::core::JointModelGroupPtr arm_joints_group_;
  moveit::core::JointModelGroupPtr leg_joints_group_;
  robot_state::RobotStatePtr goal_state_;
  bool use_stabilizing_;
  bool use_minimal_displacement_;
};

}

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_IK_H_
