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

std::vector<double> solve_ik(tf2::Transform goal, bool left_leg) {
  // the goal is the position of the sole link in the frame of the base_link

  // load URDF
  //todo do in init
  urdf::JointConstSharedPtr hip_yaw_joint, hip_pitch_joint, ankle_pitch_joint;
  urdf::Model model;
  //load URDF
  if (model.initParam("/robot_description") == 0) {
    ROS_ERROR("Loading URDF failed");
  }
  if (left_leg) {
    hip_yaw_joint = model.getJoint("LHipYaw");
    hip_pitch_joint = model.getJoint("LHipPitch");
    ankle_pitch_joint = model.getJoint("LAnklePitch");
  } else {
    hip_yaw_joint = model.getJoint("RHipYaw");
    hip_pitch_joint = model.getJoint("RHipPitch");
    ankle_pitch_joint = model.getJoint("RAnklePitch");
  }

  // first do static transform from base link to hip yaw
  tf2::Transform goal_in_hip_yaw;

  tf2::Transform trunk_to_hip_yaw;
  trunk_to_hip_yaw.setOrigin({hip_yaw_joint->parent_to_joint_origin_transform.position.x,
                              hip_yaw_joint->parent_to_joint_origin_transform.position.y,
                              hip_yaw_joint->parent_to_joint_origin_transform.position.z});
  trunk_to_hip_yaw.setRotation({0, 0, 0, 1});
  // goal in new frame
  goal_in_hip_yaw = goal * trunk_to_hip_yaw.inverse();

  // hip yaw is the only way to do yaw rotation, so it has to be the same as the goal
  double roll_goal, pitch_goal, yaw_goal;
  tf2::Matrix3x3(goal_in_hip_yaw.getRotation()).getRPY(roll_goal, pitch_goal, yaw_goal);
  float hip_yaw = yaw_goal;

  // since the axises do not cross in the hip, we are rotating the leg on a half circle
  // with the angle yaw and the radius of the distance between hip yaw and hip pitch
  // get the hip_pitch in the hip_yaw frame
  tf2::Transform hip_pitch_in_hip_yaw;
  hip_pitch_in_hip_yaw.setOrigin({hip_pitch_joint->parent_to_joint_origin_transform.position.x
                                      - hip_yaw_joint->parent_to_joint_origin_transform.position.x,
                                  hip_pitch_joint->parent_to_joint_origin_transform.position.y
                                      - hip_yaw_joint->parent_to_joint_origin_transform.position.y,
                                  hip_pitch_joint->parent_to_joint_origin_transform.position.z
                                      - hip_yaw_joint->parent_to_joint_origin_transform.position.z});
  hip_pitch_in_hip_yaw.setRotation({0, 0, 0, 1});
  // create rotation transform for yaw
  tf2::Transform yaw_rotation;
  yaw_rotation.setOrigin({0, 0, 0});
  tf2::Quaternion yaw_rotation_quat;
  yaw_rotation_quat.setRPY(0, 0, yaw_goal);
  yaw_rotation.setRotation(yaw_rotation_quat);
  // rotate the hip pitch in yaw transform
  tf2::Transform rotated_hip_pitch_in_hip_yaw = hip_pitch_in_hip_yaw * yaw_rotation;
  // get the goal in the new rotated frame of the hip pitch servo
  // we know that goal_in_hip_yaw = goal_in_hip_pitch * rotated_hip_pitch_in_hip_yaw
  // therefore
  tf2::Transform goal_in_hip_pitch = goal_in_hip_yaw * rotated_hip_pitch_in_hip_yaw.inverse();

  // now we need to substract the static transform from the last joint to the foot sole from the goal
  tf2::Transform goal_in_ankle_pitch;
  tf2::Transform ankle_goal_in_hip_pitch = goal_in_hip_pitch * goal_in_ankle_pitch.inverse();

  // now we can solve the remaining joint
  // we know that the knee only bends in one direction

  //todo remember to use the center point of the joint axis not where the motor horn is
  //todo check if forward kinematic equal to result
  //todo check if joint limits are okay
  //
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
