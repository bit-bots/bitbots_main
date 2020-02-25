#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer() {
  pid_trunk_pitch_.init(ros::NodeHandle("/walking/pid_trunk_pitch"), false);
  pid_trunk_roll_.init(ros::NodeHandle("/walking/pid_trunk_roll"), false);
  pid_trunk_fused_pitch_.init(ros::NodeHandle("/walking/pid_trunk_fused_pitch"), false);
  pid_trunk_fused_roll_.init(ros::NodeHandle("/walking/pid_trunk_fused_roll"), false);
  pid_trunk_roll_vel_.init(ros::NodeHandle("/walking/pid_trunk_gyro_roll"), false);
  pid_trunk_pitch_vel_.init(ros::NodeHandle("/walking/pid_trunk_gyro_pitch"), false);
  pid_cop_x_.init(ros::NodeHandle("/walking/pid_cop_x"), false);
  pid_cop_y_.init(ros::NodeHandle("/walking/pid_cop_y"), false);

  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.reset();
  pid_trunk_fused_pitch_.reset();
  pid_trunk_fused_roll_.reset();
  pid_trunk_roll_vel_.reset();
  pid_trunk_pitch_vel_.reset();
  pid_cop_x_.reset();
  pid_cop_y_.reset();
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
  double corrected_pitch = pid_trunk_pitch_.computeCommand(goal_pitch - response.current_pitch, dt);
  double corrected_fused_pitch = pid_trunk_fused_pitch_.computeCommand(goal_fused.fusedPitch - response.current_fused_pitch, dt);
  double corrected_roll = pid_trunk_roll_.computeCommand(goal_roll - response.current_roll, dt);
  double corrected_fused_roll = pid_trunk_fused_roll_.computeCommand(goal_fused.fusedRoll - response.current_fused_roll, dt);
  double corrected_roll_vel = pid_trunk_roll_vel_.computeCommand(response.roll_vel, dt);
  double corrected_pitch_vel = pid_trunk_pitch_vel_.computeCommand(response.pitch_vel, dt);

  tf2::Quaternion corrected_orientation;
  corrected_orientation.setRPY(goal_roll + corrected_roll + corrected_fused_roll + corrected_roll_vel, goal_pitch + corrected_pitch + corrected_fused_pitch + corrected_pitch_vel, goal_yaw);

  WalkResponse stabilized_response = response;
  stabilized_response.support_foot_to_trunk.setRotation(corrected_orientation);

  tf2::Vector3 goal_position = response.support_foot_to_trunk.getOrigin();
  double corrected_x = goal_position.x() + pid_cop_x_.computeCommand(response.sup_cop_x, dt);
  double corrected_y = goal_position.y() + pid_cop_y_.computeCommand(response.sup_cop_y, dt);

  stabilized_response.support_foot_to_trunk.setOrigin({corrected_x, corrected_y, goal_position.z()});

  return stabilized_response;
}
} // namespace bitbots_quintic_walk
