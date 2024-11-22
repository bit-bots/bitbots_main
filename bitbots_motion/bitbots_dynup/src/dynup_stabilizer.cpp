#include "bitbots_dynup/dynup_stabilizer.hpp"

namespace bitbots_dynup {

Stabilizer::Stabilizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Stabilizer params)
    : params_(params),
      pid_trunk_pitch_(node, "stabilizer.trunk_pid.pitch"),
      pid_trunk_roll_(node, "stabilizer.trunk_pid.roll") {
  pid_trunk_pitch_.initPid();
  pid_trunk_roll_.initPid();

  reset();
}

void Stabilizer::reset() {
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.reset();
}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const rclcpp::Duration &dt) {
  tf2::Transform right_foot_goal;

  // Check if we have all the necessary data to stabilize
  if (ik_goals.is_stabilizing_needed and r_sole_to_trunk_ and imu_) {
    // Convert to eigen quaternion
    tf2::Quaternion quat;
    tf2::convert(imu_->orientation, quat);
    Eigen::Quaterniond imu_orientation_eigen = Eigen::Quaterniond(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
    // Calculate fused angles
    rot_conv::FusedAngles current_orientation = rot_conv::FusedFromQuat(imu_orientation_eigen);

    // Get the goal trunk goal pose
    tf2::Transform r_sole_to_trunk_tf;
    tf2::fromMsg(r_sole_to_trunk_.value().transform, r_sole_to_trunk_tf);
    tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * r_sole_to_trunk_tf;

    tf2::Quaternion quat_msg = trunk_goal.getRotation();

    // Calculate fused angles for the trunk goal orientation
    Eigen::Quaterniond goal_orientation_eigen =
        Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());
    rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

    // Adapt trunk based on PID controller
    goal_fused.fusedPitch +=
        pid_trunk_pitch_.computeCommand(goal_fused.fusedPitch - current_orientation.fusedPitch, dt);
    goal_fused.fusedRoll += pid_trunk_roll_.computeCommand(goal_fused.fusedRoll - current_orientation.fusedRoll, dt);

    // Check if the trunk is stable, meaning it isn't leaning too much
    // TODO it would be better to use the rotational velocity of the imu to determine stability
    is_stable_ = (abs(goal_fused.fusedPitch - current_orientation.fusedPitch) < params_.end_pause.stable_threshold) &&
                 (abs(goal_fused.fusedRoll - current_orientation.fusedRoll) < params_.end_pause.stable_threshold);

    // Convert the fused angles back to a quaternion
    tf2::Quaternion corrected_orientation;
    Eigen::Quaterniond goal_orientation_eigen_corrected = rot_conv::QuatFromFused(goal_fused);
    tf2::convert(goal_orientation_eigen_corrected, corrected_orientation);

    // Change the trunk goal orientation
    trunk_goal.setRotation(corrected_orientation);
    // Then calculate how the foot should be placed to reach that trunk pose
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

void Stabilizer::setParams(bitbots_dynup::Params::Stabilizer params) { params_ = params; }

bool Stabilizer::isStable() { return is_stable_; }

void Stabilizer::setImu(sensor_msgs::msg::Imu::SharedPtr imu) { imu_ = imu; }

void Stabilizer::setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk) {
  r_sole_to_trunk_ = r_sole_to_trunk;
}

}  // namespace bitbots_dynup
