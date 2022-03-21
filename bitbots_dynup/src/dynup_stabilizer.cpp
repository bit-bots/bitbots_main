#include "bitbots_dynup/dynup_stabilizer.h"

namespace bitbots_dynup {

Stabilizer::Stabilizer(std::string ns){
    pitch_node_ = rclcpp::Node::make_shared(ns + "pid_trunk_fused_pitch");
    roll_node_ = rclcpp::Node::make_shared(ns + "pid_trunk_fused_roll");

    pitch_node_->declare_parameter<double>("p", 0.0);
    pitch_node_->declare_parameter<double>("i", 0.0);
    pitch_node_->declare_parameter<double>("d", 0.0);
    pitch_node_->declare_parameter<double>("i_clamp_max", 0.0);
    pitch_node_->declare_parameter<double>("i_clamp_min", 0.0);
    pitch_node_->declare_parameter<bool>("antiwindup", false);
    roll_node_->declare_parameter<double>("p", 0.0);
    roll_node_->declare_parameter<double>("i", 0.0);
    roll_node_->declare_parameter<double>("d", 0.0);
    roll_node_->declare_parameter<double>("i_clamp_max", 0.0);
    roll_node_->declare_parameter<double>("i_clamp_min", 0.0);
    roll_node_->declare_parameter<bool>("antiwindup", false);

    pid_trunk_pitch_ = std::make_shared<control_toolbox::PidROS>(pitch_node_, "");
    pid_trunk_roll_ = std::make_shared<control_toolbox::PidROS>(roll_node_, "");
    pid_trunk_pitch_->initPid();
    pid_trunk_roll_->initPid();

    reset();
}

void Stabilizer::reset() {
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.reset();

}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const rclcpp::Duration &dt) {
  spin_some(pitch_node_);
  spin_some(roll_node_);

  tf2::Transform right_foot_goal;
  tf2::Quaternion quat;
  tf2::convert(imu_->orientation, quat);

  Eigen::Quaterniond imu_orientation_eigen = Eigen::Quaterniond(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
  rot_conv::FusedAngles current_orientation = rot_conv::FusedFromQuat(imu_orientation_eigen);

  if (use_stabilizing_ && ik_goals.is_stabilizing_needed) {
    tf2::Transform r_sole_to_trunk_tf;
    tf2::fromMsg(r_sole_to_trunk_.transform, r_sole_to_trunk_tf);
    tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * r_sole_to_trunk_tf;

    // compute orientation with fused angles for PID control
    tf2::Quaternion quat_msg = trunk_goal.getRotation();
    Eigen::Quaterniond goal_orientation_eigen = Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());
    rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

    // adapt trunk based on PID controller
    goal_fused.fusedPitch +=
        pid_trunk_pitch_->computeCommand(goal_fused.fusedPitch - current_orientation.fusedPitch, dt);
    goal_fused.fusedRoll += pid_trunk_roll_->computeCommand(goal_fused.fusedRoll - current_orientation.fusedRoll, dt);

    is_stable_ = (abs(goal_fused.fusedPitch - current_orientation.fusedPitch) < stable_threshold_) &&
        (abs(goal_fused.fusedRoll - current_orientation.fusedRoll) < stable_threshold_);

    tf2::Quaternion corrected_orientation;
    Eigen::Quaterniond goal_orientation_eigen_corrected = rot_conv::QuatFromFused(goal_fused);
    tf2::convert(goal_orientation_eigen_corrected, corrected_orientation);
    trunk_goal.setRotation(corrected_orientation);

    // then calculate how the foot should be placed to reach that trunk pose
    right_foot_goal = trunk_goal * r_sole_to_trunk_tf.inverse();
  } else {
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
  response.is_head_zero = ik_goals.is_head_zero;

  return response;
}

void Stabilizer::setParams(std::map<std::string, rclcpp::Parameter> params) {
  use_stabilizing_ = params["stabilizing"].get_value<bool>();
  stable_threshold_ = params["stable_threshold"].get_value<double>();

}

bool Stabilizer::isStable() {
  return is_stable_;
}

void Stabilizer::setImu(sensor_msgs::msg::Imu::SharedPtr imu) {
  imu_ = imu;
}

void Stabilizer::setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk) {
  r_sole_to_trunk_ = r_sole_to_trunk;
}

}
