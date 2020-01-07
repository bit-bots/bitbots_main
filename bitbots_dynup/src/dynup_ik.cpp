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

bitbots_splines::JointGoals DynupIK::calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) {
  double bio_ik_timeout = 0.01;

  auto ik_options_arms = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  ik_options_arms->replace = true;
  ik_options_arms->return_approximate_solution = true;
  auto ik_options_legs = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  ik_options_legs->replace = true;
  ik_options_legs->return_approximate_solution = true;

  ik_options_legs->goals.emplace_back(ik_goals->goals[0].release());
  ik_options_legs->goals.emplace_back(ik_goals->goals[1].release());
  ik_options_arms->goals.emplace_back(ik_goals->goals[2].release());
  ik_options_arms->goals.emplace_back(ik_goals->goals[3].release());
  ik_options_legs->goals.emplace_back(ik_goals->goals[4].release());
  //TODO displacement goal
  //TODO what happens if one of the goals is missing?

  bool success = goal_state_->setFromIK(arm_joints_group_.get(),
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        *ik_options_arms) &&
                 goal_state_->setFromIK(leg_joints_group_.get(),
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        *ik_options_legs);
  

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

}
