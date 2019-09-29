#include <bitbots_dynup/dynup_ik.h>
namespace bitbots_dynup {
void DynupIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  legs_joints_group_.reset(kinematic_model->getJointModelGroup("Legs"));

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  goal_state_->setToDefaultValues();
}

void DynupIK::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
  for (int i = 0; i < names_vec.size(); ++i) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

bitbots_splines::JointGoals DynupIK::calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) {

  double bio_ik_timeout = 0.01;
  bool success = goal_state_->setFromIK(legs_joints_group_.get(),
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        *ik_goals);

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_.get(), joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  } else {
    return {};
  }
}
}
