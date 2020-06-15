#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer() {
  pid_trunk_fused_pitch_.init(ros::NodeHandle("walking/pid_trunk_fused_pitch"), false);
  pid_trunk_fused_roll_.init(ros::NodeHandle("walking/pid_trunk_fused_roll"), false);

  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_fused_pitch_.reset();
  pid_trunk_fused_roll_.reset();
}

WalkResponse WalkStabilizer::stabilize(const WalkResponse &response, const ros::Duration &dt) {
  // compute orientation with PID control
  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);

  Eigen::Quaterniond goal_orientation_eigen;
  tf2::convert(response.support_foot_to_trunk.getRotation(), goal_orientation_eigen);

  // compute orientation with fused angles for PID control
  rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

  // adapt trunk values based on PID controllers
  double fused_roll_correction = pid_trunk_fused_roll_.computeCommand(goal_fused.fusedRoll - response.current_fused_roll, dt);
  double fused_pitch_correction = pid_trunk_fused_pitch_.computeCommand(goal_fused.fusedPitch - response.current_fused_pitch, dt);

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
