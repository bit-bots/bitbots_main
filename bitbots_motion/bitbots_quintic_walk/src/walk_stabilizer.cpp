#include "bitbots_quintic_walk/walk_stabilizer.hpp"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer(rclcpp::Node::SharedPtr node)
    : pid_trunk_fused_pitch_(node, "node.trunk_pid.pitch"),
      pid_trunk_fused_roll_(node, "node.trunk_pid.roll"),
      pid_step_length_adjustment_pitch_(node, "node.step_length.pitch"),
      pid_step_length_adjustment_roll_(node, "node.step_length.roll") {
  pid_step_length_adjustment_pitch_.initPid();
  pid_step_length_adjustment_roll_.initPid();

  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_fused_pitch_.reset();
  pid_trunk_fused_roll_.reset();
  pid_step_length_adjustment_pitch_.reset();
  pid_step_length_adjustment_roll_.reset();
}

WalkRequest WalkStabilizer::adjust_step_length(WalkRequest request, const double imu_roll, const double imu_pitch,
                                               double pitch_threshold, double roll_threshold,
                                               const rclcpp::Duration& dt) {
  double adjustment_pitch = pid_step_length_adjustment_pitch_.computeCommand(imu_roll, dt);
  double adjustment_roll = pid_step_length_adjustment_roll_.computeCommand(imu_roll, dt);
  // adapt step length values based on PID controllers
  // we use a threshold to avoid unneeded steps
  if (adjustment_pitch >= pitch_threshold) {
    request.linear_orders[0] += adjustment_pitch;
  }
  if (adjustment_roll >= roll_threshold) {
    request.linear_orders[1] += adjustment_roll;
  }
  return request;
}

WalkResponse WalkStabilizer::stabilize(const WalkResponse& response, const rclcpp::Duration& dt) {
  // compute orientation with PID control
  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);

  tf2::Quaternion quat_msg = response.support_foot_to_trunk.getRotation();
  Eigen::Quaterniond goal_orientation_eigen =
      Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());

  // compute orientation with fused angles for PID control
  rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

  // adapt trunk values based on PID controllers
  double fused_roll_correction =
      pid_trunk_fused_roll_.computeCommand(goal_fused.fusedRoll - response.current_fused_roll, dt);
  double fused_pitch_correction =
      pid_trunk_fused_pitch_.computeCommand(goal_fused.fusedPitch - response.current_fused_pitch, dt);

  // Change trunk x offset (in the trunks frame of reference) based on the PID controllers
  WalkResponse stabilized_response{response};

  tf2::Transform trunk_offset;
  trunk_offset.setOrigin(tf2::Vector3(fused_pitch_correction, fused_roll_correction, 0.0));
  trunk_offset.setRotation(tf2::Quaternion::getIdentity());

  // apply the trunk offset to the trunk pose
  stabilized_response.support_foot_to_trunk = trunk_offset * response.support_foot_to_trunk;

  return stabilized_response;
}
}  // namespace bitbots_quintic_walk
