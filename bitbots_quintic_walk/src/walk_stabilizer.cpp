#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer(){
  pitch_node_ = rclcpp::Node::make_shared("pid_trunk_fused_pitch");
  roll_node_ = rclcpp::Node::make_shared("pid_trunk_fused_roll");

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

  pid_trunk_fused_pitch_ = std::make_shared<control_toolbox::PidROS>(pitch_node_, "");
  pid_trunk_fused_roll_ = std::make_shared<control_toolbox::PidROS>(roll_node_, "");
  pid_trunk_fused_pitch_->initPid();
  pid_trunk_fused_roll_->initPid();

  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_fused_pitch_->reset();
  pid_trunk_fused_roll_->reset();
}

WalkResponse WalkStabilizer::stabilize(const WalkResponse &response, const rclcpp::Duration &dt) {
  // let pid nodes spine
  spin_some(pitch_node_);
  spin_some(roll_node_);

  // compute orientation with PID control
  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);

  tf2::Quaternion quat_msg = response.support_foot_to_trunk.getRotation();
  Eigen::Quaterniond goal_orientation_eigen = Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());

  // compute orientation with fused angles for PID control
  rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

  // adapt trunk values based on PID controllers
  double fused_roll_correction =
      pid_trunk_fused_roll_->computeCommand(goal_fused.fusedRoll - response.current_fused_roll, dt);
  double fused_pitch_correction =
      pid_trunk_fused_pitch_->computeCommand(goal_fused.fusedPitch - response.current_fused_pitch, dt);

  tf2::Quaternion corrected_orientation;
  goal_fused.fusedRoll += fused_roll_correction;
  goal_fused.fusedPitch += fused_pitch_correction;
  Eigen::Quaterniond goal_orientation_eigen_corrected = rot_conv::QuatFromFused(goal_fused);
  tf2::convert(goal_orientation_eigen_corrected, corrected_orientation);

  WalkResponse stabilized_response = response;
  stabilized_response.support_foot_to_trunk.setRotation(corrected_orientation);

  return stabilized_response;
}
} // namespace bitbots_quintic_walk
