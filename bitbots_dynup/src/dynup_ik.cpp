#include <bitbots_dynup/dynup_ik.h>
namespace bitbots_dynup {
void DynupIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  l_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
  r_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");
  l_arm_joints_group_ = kinematic_model->getJointModelGroup("LeftArm");
  r_arm_joints_group_ = kinematic_model->getJointModelGroup("RightArm");
  all_joints_group_ = kinematic_model->getJointModelGroup("All");

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  //goal_state_->setToDefaultValues();

  const Eigen::Isometry3d &end_effector_state = goal_state_->getGlobalLinkTransform("r_sole");
}

void DynupIK::reset() {
  for (int i = 0; i < currentJointStates.name.size(); i++) {
      goal_state_->setJointPositions(currentJointStates.name[i], &currentJointStates.position[i]);
  }
}

bitbots_splines::JointGoals DynupIK::calculate(const DynupResponse &ik_goals) {

  /* ik options is basically the command which we send to bio_ik and which describes what we want to do */
  auto ik_options = kinematics::KinematicsQueryOptions();
  ik_options.return_approximate_solution = true;

  geometry_msgs::Pose right_foot_goal_msg, left_foot_goal_msg, right_hand_goal_msg, left_hand_goal_msg;
  
  tf2::toMsg(ik_goals.r_foot_goal_pose, right_foot_goal_msg);
  tf2::toMsg(ik_goals.l_foot_goal_pose, left_foot_goal_msg);
  tf2::toMsg(ik_goals.r_hand_goal_pose, right_hand_goal_msg);
  tf2::toMsg(ik_goals.l_hand_goal_pose, left_hand_goal_msg);


  bool success;
  goal_state_->updateLinkTransforms();

  success = goal_state_->setFromIK(l_leg_joints_group_,
                                   left_foot_goal_msg,
                                   0.001,
                                   moveit::core::GroupStateValidityCallbackFn(),
                                   ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(r_leg_joints_group_,
                                   right_foot_goal_msg,
                                   0.001,
                                   moveit::core::GroupStateValidityCallbackFn(),
                                   ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(l_arm_joints_group_,
                                   left_hand_goal_msg,
                                   0.001,
                                   moveit::core::GroupStateValidityCallbackFn(),
                                   ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(r_arm_joints_group_,
                                   right_hand_goal_msg,
                                   0.001,
                                   moveit::core::GroupStateValidityCallbackFn(),
                                   ik_options);

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = all_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(all_joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    /* sets head motor positions to 0, as the IK will return random values for those unconstrained motors. */
    for(int i = 0; i  < result.first.size(); i++)
    {
        if(result.first[i] == "HeadPan"||result.first[i] ==  "HeadTilt") {
            result.second[i] = 0;
        }
    }
    return result;
  } else {
    // node will count this as a missing tick and provide warning
    return {};
  }
}
void DynupIK::useStabilizing(bool use) {
  use_stabilizing_ = use;
}
}
