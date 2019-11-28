#include "bitbots_quintic_walk/walk_ik.h"

namespace bitbots_quintic_walk {

WalkIK::WalkIK() : bio_ik_timeout_(0.01) {}

void WalkIK::init(moveit::core::RobotModelPtr kinematic_model) {
  legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");
  left_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
  right_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");

  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  goal_state_->setToDefaultValues();

  reset();
}

bitbots_splines::JointGoals WalkIK::calculateDirectly(const WalkResponse &ik_goals) {
  SWRI_PROFILE("IK-direct");

  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;

  // make pose msg for calling IK
  geometry_msgs::Pose left_foot_goal_msg;
  geometry_msgs::Pose right_foot_goal_msg;

  // decide which foot is which
  if (ik_goals.is_left_support_foot) {
    tf2::toMsg(trunk_to_support_foot_goal, left_foot_goal_msg);
    tf2::toMsg(trunk_to_flying_foot_goal, right_foot_goal_msg);
  } else {
    tf2::toMsg(trunk_to_support_foot_goal, left_foot_goal_msg);
    tf2::toMsg(trunk_to_flying_foot_goal, right_foot_goal_msg);
  }
  const geometry_msgs::Pose left_foot_goal_msg_const = left_foot_goal_msg_const;
  const geometry_msgs::Pose right_foot_goal_msg_const = right_foot_goal_msg_const;

  bool success;
  {
    SWRI_PROFILE("IK-direct-call");
    success = goal_state_->setFromIK(left_leg_joints_group_,
                                     left_foot_goal_msg_const,
                                     bio_ik_timeout_);
    ROS_WARN("success1 %d", success);
    success = goal_state_->setFromIK(right_leg_joints_group_,
                                     right_foot_goal_msg_const,
                                     bio_ik_timeout_);
    ROS_WARN("success2 %d", success);
  }

  std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
  std::vector<double> joint_goals;
  goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

  /* construct result object */
  bitbots_splines::JointGoals result;
  result.first = joint_names;
  result.second = joint_goals;
  return result;
}

bitbots_splines::JointGoals WalkIK::calculate(const std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) {
  SWRI_PROFILE("IK");
  bool success;
  {
    SWRI_PROFILE("IK-call");
    success = goal_state_->setFromIK(legs_joints_group_,
                                     EigenSTL::vector_Isometry3d(),
                                     std::vector<std::string>(),
                                     bio_ik_timeout_,
                                     moveit::core::GroupStateValidityCallbackFn(),
                                     *ik_goals);
  }

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  } else {
    /* maybe do something better here? */
    return bitbots_splines::JointGoals();
  }
}

void WalkIK::reset() {
  // we have to set some good initial position in the goal state, since we are using a gradient
  // based method. Otherwise, the first step will be not correct
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
  for (int i = 0; i < names_vec.size(); i++) {
    // besides its name, this method only changes a single joint position...
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

void WalkIK::setBioIKTimeout(double timeout) {
  bio_ik_timeout_ = timeout;
};

}
