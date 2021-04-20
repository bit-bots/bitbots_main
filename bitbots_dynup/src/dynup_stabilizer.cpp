#include "bitbots_dynup/dynup_stabilizer.h"

#include <bitbots_splines/dynamic_balancing_goal.h>


namespace bitbots_dynup {

void Stabilizer::init(moveit::core::RobotModelPtr kinematic_model) {
  kinematic_model_ = std::move(kinematic_model);
  ros::NodeHandle nhp = ros::NodeHandle("dynup/pid_trunk_pitch");
  ros::NodeHandle nhr = ros::NodeHandle("dynup/pid_trunk_roll");

  pid_trunk_pitch_.init(nhp, false);
  pid_trunk_roll_.init(nhr, false);

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model_));
  goal_state_->setToDefaultValues();
}

void Stabilizer::reset() {
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.reset();

}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const ros::Duration &dt) {
    tf2::Transform right_foot_goal;
    tf2::Quaternion quat;
    tf2::convert(imu_.orientation, quat);

    Eigen::Quaterniond imu_orientation_eigen;
    tf2::convert(quat, imu_orientation_eigen);
    rot_conv::FusedAngles current_orientation = rot_conv::FusedFromQuat(imu_orientation_eigen);

    if(use_stabilizing_ && ik_goals.is_stabilizing_needed) {
        tf2::Transform r_sole_to_trunk_tf;
        tf2::fromMsg(r_sole_to_trunk_.transform, r_sole_to_trunk_tf);
        tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * r_sole_to_trunk_tf;

        // compute orientation with fused angles for PID control
        Eigen::Quaterniond goal_orientation_eigen;
        tf2::convert(trunk_goal.getRotation(), goal_orientation_eigen);
        rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

        // adapt trunk based on PID controller
        goal_fused.fusedPitch += pid_trunk_pitch_.computeCommand(goal_fused.fusedPitch - current_orientation.fusedPitch, dt);
        goal_fused.fusedRoll += pid_trunk_roll_.computeCommand(goal_fused.fusedRoll - current_orientation.fusedRoll, dt);

        is_stable_ = (abs(goal_fused.fusedPitch - current_orientation.fusedPitch) < stable_threshold_) &&
                (abs(goal_fused.fusedRoll - current_orientation.fusedRoll) < stable_threshold_);

        tf2::Quaternion corrected_orientation;
        Eigen::Quaterniond goal_orientation_eigen_corrected = rot_conv::QuatFromFused(goal_fused);
        tf2::convert(goal_orientation_eigen_corrected, corrected_orientation);
        trunk_goal.setRotation(corrected_orientation);

        // then calculate how the foot should be placed to reach that trunk pose
        right_foot_goal = trunk_goal * r_sole_to_trunk_tf.inverse() ;
    }
    else {
        right_foot_goal = ik_goals.r_foot_goal_pose;
    }

    tf2::Transform left_foot_goal = ik_goals.l_foot_goal_pose * right_foot_goal;
    tf2::Transform left_hand_goal = ik_goals.l_hand_goal_pose;
    tf2::Transform right_hand_goal = ik_goals.r_hand_goal_pose;

    DynupResponse response;
    response.r_foot_goal_pose = right_foot_goal;
    response.l_foot_goal_pose = left_foot_goal;
    response.r_hand_goal_pose = right_hand_goal;
    response.l_hand_goal_pose = left_hand_goal;

    return response;
}

void Stabilizer::setParams(DynUpConfig params) {
  use_stabilizing_ = params.stabilizing;
  stable_threshold_ = params.stable_threshold;

}

bool Stabilizer::isStable() {
    return is_stable_;
}


void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

void Stabilizer::setImu(sensor_msgs::Imu imu) {
  imu_ = imu;
}


void Stabilizer::setRSoleToTrunk(geometry_msgs::TransformStamped r_sole_to_trunk) {
  r_sole_to_trunk_ = r_sole_to_trunk;
}

}
