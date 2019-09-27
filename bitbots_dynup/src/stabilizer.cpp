#include "bitbots_dynup/stabilizer.h"

#include <memory>
#include "bitbots_dynup/dynamic_balancing_goal.h"
#include "bitbots_dynup/reference_goals.h"

namespace bitbots_dynup {

Stabilizer::Stabilizer() {
  /* load MoveIt! model */
  robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
  robot_model_loader.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());

  /* Extract joint groups from loaded model */
  kinematic_model_ = robot_model_loader.getModel();
  legs_joints_group_ = kinematic_model_->getJointModelGroup("Legs");

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model_));
  goal_state_->setToDefaultValues();

  reset();
}

void Stabilizer::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec =
      {"HeadPan", "HeadTilt", "LAnklePitch", "LAnkleRoll", "LElbow", "LHipPitch", "LHipRoll", "LHipYaw", "LKnee",
       "LShoulderPitch", "LShoulderRoll", "RAnklePitch", "RAnkleRoll", "RElbow", "RHipPitch", "RHipRoll", "RHipYaw",
       "RKnee", "RShoulderPitch", "RShoulderRoll"};
  std::vector<double> pos_vec = {0, 47, -80, 0, -88, 65, -7, 2, -157, 11, -14, 80, 1, 97, -63, -3, 0, 157, -14, 14};
  for (double &pos: pos_vec) {
    pos = pos / 180.0 * M_PI;
  }
  for (int i = 0; i < names_vec.size(); i++) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

std::optional<JointGoals> Stabilizer::stabilize(geometry_msgs::Point support_point,
                                                geometry_msgs::PoseStamped &l_foot_goal_pose,
                                                geometry_msgs::PoseStamped &trunk_goal_pose) {
  /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.replace = true;
  ik_options.return_approximate_solution = true;
  double bio_ik_timeout = 0.01;

  tf2::Stamped<tf2::Transform> tf_l_foot, tf_trunk;
  tf2::convert(l_foot_goal_pose, tf_l_foot);
  tf2::convert(trunk_goal_pose, tf_trunk);

  /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
  auto *bio_ik_l_foot_goal = new ReferencePoseGoal();
  bio_ik_l_foot_goal->setPosition(tf_l_foot.getOrigin());
  bio_ik_l_foot_goal->setOrientation(tf_l_foot.getRotation());
  bio_ik_l_foot_goal->setLinkName("l_sole");
  bio_ik_l_foot_goal->setWeight(1.0);
  bio_ik_l_foot_goal->setReferenceLinkName("r_sole");

  auto *bio_ik_trunk_goal = new ReferencePoseGoal();
  bio_ik_trunk_goal->setPosition(tf_trunk.getOrigin());
  bio_ik_trunk_goal->setOrientation(tf_trunk.getRotation());
  bio_ik_trunk_goal->setLinkName("torso");
  bio_ik_trunk_goal->setWeight(1.0);
  bio_ik_trunk_goal->setReferenceLinkName("r_sole");

  tf2::Vector3 stabilizing_target = {support_point.x, support_point.y, support_point.z};
  DynamicBalancingContext bio_ik_balancing_context(kinematic_model_);
  auto *bio_ik_balance_goal =
      new DynamicBalancingGoal(&bio_ik_balancing_context, stabilizing_target, stabilizing_weight_);
  bio_ik_balance_goal->setReferenceLink("base_link");

  ik_options.goals.emplace_back(bio_ik_l_foot_goal);
  ik_options.goals.emplace_back(bio_ik_trunk_goal);

  if (use_stabilizing_ && false) {
    ik_options.goals.emplace_back(bio_ik_balance_goal);
  }
  if (use_minimal_displacement_ && false) {
    ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal());
  }

  bool success = goal_state_->setFromIK(legs_joints_group_,
                                        EigenSTL::vector_Isometry3d(),
                                        std::vector<std::string>(),
                                        bio_ik_timeout,
                                        moveit::core::GroupStateValidityCallbackFn(),
                                        ik_options);

  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  } else {
    return std::nullopt;
  }
}

void Stabilizer::useStabilizing(bool use) {
  use_stabilizing_ = use;
}

void Stabilizer::useMinimalDisplacement(bool use) {
  use_minimal_displacement_ = use;
}

void Stabilizer::setStabilizingWeight(double weight) {
  stabilizing_weight_ = weight;
}

}
