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
  std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
  for (int i = 0; i < names_vec.size(); ++i) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

bitbots_splines::JointGoals KickIK::calculate(const KickPositions &positions) {
  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = positions.trunk_pose.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * positions.flying_foot_pose;

  // make pose msg for calling IK
  geometry_msgs::Pose left_foot_goal_msg;
  geometry_msgs::Pose right_foot_goal_msg;

  // decide which foot is which
  if (positions.is_left_kick) {
    tf2::toMsg(trunk_to_support_foot_goal, right_foot_goal_msg);
    tf2::toMsg(trunk_to_flying_foot_goal, left_foot_goal_msg);
  } else {
    tf2::toMsg(trunk_to_support_foot_goal, left_foot_goal_msg);
    tf2::toMsg(trunk_to_flying_foot_goal, right_foot_goal_msg);
  }

  // call IK two times, since we have two legs
  bool success;

  // we have to do this otherwise there is an error
  goal_state_->updateLinkTransforms();

  success = goal_state_->setFromIK(left_leg_joints_group_,
                                   left_foot_goal_msg,
                                   0.01,
                                   moveit::core::GroupStateValidityCallbackFn());
  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(right_leg_joints_group_,
                                    right_foot_goal_msg,
                                    0.01,
                                    moveit::core::GroupStateValidityCallbackFn());

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
