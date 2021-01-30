#include "bitbots_dynamic_kick/kick_ik.h"

namespace bitbots_dynamic_kick {

void KickIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");
  left_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
  right_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  goal_state_->setToDefaultValues();

  /* Initialize collision model */
  planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model));
}

void KickIK::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, 1.0, -0.4, -0.7, -1.0, 0.4};
  for (int i = 0; i < names_vec.size(); ++i) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

bitbots_splines::JointGoals KickIK::calculate(const KickPositions &positions) {
  // change goals from support foot based coordinate system to trunk based coordinate system
  Eigen::Isometry3d trunk_to_support_foot_goal = positions.trunk_pose.inverse();
  Eigen::Isometry3d trunk_to_flying_foot_goal = trunk_to_support_foot_goal * positions.flying_foot_pose;

  // decide which foot is which
  robot_model::JointModelGroup *support_foot_group, *flying_foot_group;
  if (positions.is_left_kick) {
    support_foot_group = right_leg_joints_group_;
    flying_foot_group = left_leg_joints_group_;
  } else {
    support_foot_group = left_leg_joints_group_;
    flying_foot_group = right_leg_joints_group_;
  }

  // we have to do this otherwise there is an error
  goal_state_->updateLinkTransforms();

  // call IK two times, since we have two legs
  // we don't listen to the return value since BioIK always returns true
  goal_state_->setFromIK(support_foot_group,
                         trunk_to_support_foot_goal,
                         0.01);
  goal_state_->updateLinkTransforms();

  goal_state_->setFromIK(flying_foot_group,
                         trunk_to_flying_foot_goal,
                         0.01);

  std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
  std::vector<double> joint_goals;
  goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

  /* construct result object */
  bitbots_splines::JointGoals result;
  result.first = joint_names;
  result.second = joint_goals;
  return result;
}

}
