#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_

#include <Eigen/Geometry>
#include <bitbots_dynamic_kick/kick_utils.hpp>
#include <bitbots_splines/abstract_ik.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

namespace bitbots_dynamic_kick {

class KickIK : public bitbots_splines::AbstractIK<KickPositions> {
 public:
  KickIK() : legs_joints_group_(), left_leg_joints_group_(), right_leg_joints_group_(){};
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  bitbots_splines::JointGoals calculate(const KickPositions &positions) override;
  void reset() override;

 private:
  moveit::core::RobotStatePtr goal_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::JointModelGroup *legs_joints_group_;
  moveit::core::JointModelGroup *left_leg_joints_group_;
  moveit::core::JointModelGroup *right_leg_joints_group_;
};
}  // namespace bitbots_dynamic_kick

#endif  // BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_IK_H_
