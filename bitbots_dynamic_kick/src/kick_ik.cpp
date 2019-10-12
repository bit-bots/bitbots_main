#include "bitbots_dynamic_kick/kick_ik.h"
#include <bitbots_splines/dynamic_balancing_goal.h>

namespace bitbots_dynamic_kick {

void KickIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  m_legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");

  /* Reset kinematic goal to default */
  m_goal_state_.reset(new robot_state::RobotState(kinematic_model));
  m_goal_state_->setToDefaultValues();

  /* Initialize collision model */
  m_planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model));
}

void KickIK::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
  for (int i = 0; i < names_vec.size(); ++i) {
    m_goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

bitbots_splines::JointGoals KickIK::calculate(const std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) {
  double bio_ik_timeout = 0.01;
  bool success = m_goal_state_->setFromIK(m_legs_joints_group_,
                                          EigenSTL::vector_Isometry3d(),
                                          std::vector<std::string>(),
                                          bio_ik_timeout,
                                          moveit::core::GroupStateValidityCallbackFn(),
                                          *ik_goals);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm = m_planning_scene_->getAllowedCollisionMatrix();
  m_planning_scene_->checkCollision(req, res, *m_goal_state_, acm);
  if (res.collision) {
    ROS_ERROR_STREAM("Aborting due to self collision!");
    success = false;
  }

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = m_legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    m_goal_state_->copyJointGroupPositions(m_legs_joints_group_, joint_goals);

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

}
