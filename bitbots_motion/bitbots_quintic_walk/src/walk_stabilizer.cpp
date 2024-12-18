#include <bitbots_quintic_walk/walk_stabilizer.hpp>

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer(rclcpp::Node::SharedPtr node)
    : pid_trunk_fused_pitch_(node, "node.trunk_pid.pitch"), pid_trunk_fused_roll_(node, "node.trunk_pid.roll") {
  pid_trunk_fused_pitch_.initPid();
  pid_trunk_fused_roll_.initPid();

  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_fused_pitch_.reset();
  pid_trunk_fused_roll_.reset();
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
