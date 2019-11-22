#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_

#include "bitbots_splines/abstract_ik.h"
namespace bitbots_quintic_walk {

class WalkIK : public bitbots_splines::AbstractIK {
 public:
  WalkIK();

  bitbots_splines::JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setBioIKTimeout(double timeout);

 private:
  robot_state::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup *legs_joints_group_;

  double bio_ik_timeout_;

};
}
#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_IK_H_
