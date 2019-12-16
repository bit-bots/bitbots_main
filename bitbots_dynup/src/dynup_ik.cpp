#include <bitbots_dynup/dynup_ik.h>
namespace bitbots_dynup {
void DynupIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  all_joints_group_.reset(kinematic_model->getJointModelGroup("All"));
  arm_joints_group_.reset(kinematic_model->getJointModelGroup("Arms"));
  leg_joints_group_.reset(kinematic_model->getJointModelGroup("Legs"));



  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  goal_state_->setToDefaultValues();

  const Eigen::Isometry3d &end_effector_state = goal_state_->getGlobalLinkTransform("r_sole");
}

void DynupIK::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec =
      {"HeadPan", "HeadTilt", "LAnklePitch", "LAnkleRoll", "LElbow", "LHipPitch", "LHipRoll", "LHipYaw", "LKnee",
       "LShoulderPitch", "LShoulderRoll", "RAnklePitch", "RAnkleRoll", "RElbow", "RHipPitch", "RHipRoll", "RHipYaw",
       "RKnee", "RShoulderPitch", "RShoulderRoll"};
  std::vector<double> pos_vec = {0, 0, -25, 4, 45, 27, 4, -1, -58, 0, 25, -4, 45, -37, -4, 1, 0, 58, 0, 0};
  for (double &pos: pos_vec) {
    pos = pos / 180.0 * M_PI;
  }
  for (int i = 0; i < names_vec.size(); i++) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

bitbots_splines::JointGoals DynupIK::calculate(const DynupResponse &ik_goals) {

  tf2::Transform right_foot_goal = ik_goals.r_foot_goal_pose;
  tf2::Transform left_foot_goal = ik_goals.l_foot_goal_pose * ik_goals.r_foot_goal_pose;
  tf2::Transform left_hand_goal = ik_goals.l_hand_goal_pose;
  tf2::Transform right_hand_goal = ik_goals.r_hand_goal_pose;

  geometry_msgs::Pose right_foot_goal_msg, left_foot_goal_msg, right_hand_goal_msg, left_hand_goal_msg;

  tf2::toMsg(right_foot_goal, right_foot_goal_msg);
  tf2::toMsg(left_foot_goal, left_foot_goal_msg);
  tf2::toMsg(right_hand_goal, right_hand_goal_msg);
  tf2::toMsg(left_hand_goal, left_hand_goal_msg);


  bool success;

  goal_state_->updateLinkTransforms();

  success = goal_state_->setFromIK(l_leg_joints_group_,
                                   left_foot_goal_msg,
                                   0.01,
                                   moveit::core::GroupStateValidityCallbackFn());
  goal_state_->updateLinkTransforms();
  //ROS_ERROR("1 %d", success);

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

bitbots_splines::JointGoals DynupIK::calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) {
  double bio_ik_timeout = 0.01;
  bool success = goal_state_->setFromIK(arm_joints_group_.get(),
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        *ik_goals) &&
                 goal_state_->setFromIK(leg_joints_group_.get(),
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        *ik_goals);
  ;

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = all_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(all_joints_group_.get(), joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  } else {
    return {};
  }
}
void DynupIK::useStabilizing(bool use) {
  use_stabilizing_ = use;
}

void DynupIK::useMinimalDisplacement(bool use) {
  use_minimal_displacement_ = use;
}
}
