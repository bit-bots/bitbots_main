#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <bitbots_splines/abstract_ik.h>

namespace bitbots_dynamic_kick {

class KickIK : public bitbots_splines::AbstractIK {
 public:
  KickIK() = default;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  bitbots_splines::JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
  void reset() override;

 private:
  robot_state::RobotStatePtr m_goal_state_;
  planning_scene::PlanningScenePtr m_planning_scene_;
  robot_model::JointModelGroup *m_legs_joints_group_;
};
}

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
