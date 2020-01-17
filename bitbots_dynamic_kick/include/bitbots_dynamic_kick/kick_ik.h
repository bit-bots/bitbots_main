#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <bitbots_splines/abstract_ik.h>
#include <bitbots_dynamic_kick/kick_utils.h>

namespace bitbots_dynamic_kick {

class KickIK : public bitbots_splines::AbstractIK {
 public:
  KickIK() = default;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  bitbots_splines::JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
  bitbots_splines::JointGoals calculateDirectly(KickPositions &positions);
  void reset() override;

 private:
  robot_state::RobotStatePtr goal_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  robot_model::JointModelGroup *legs_joints_group_;
  robot_model::JointModelGroup *left_leg_joints_group_;
  robot_model::JointModelGroup *right_leg_joints_group_;
};
}

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
