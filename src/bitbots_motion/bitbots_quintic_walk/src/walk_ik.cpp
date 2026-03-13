#include <bitbots_quintic_walk/walk_ik.hpp>

namespace bitbots_quintic_walk {

WalkIK::WalkIK(rclcpp::Node::SharedPtr node, walking::Params::Node::Ik config) : node_(node), config_(config) {}

void WalkIK::init(moveit::core::RobotModelPtr kinematic_model) {
  if (config_.reset) {
    reset();
  }
}

bitbots_splines::JointGoals WalkIK::calculate(const WalkResponse& ik_goals) {
  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;

  // make pose msg for calling IK
  geometry_msgs::msg::Transform left_foot_goal, right_foot_goal;

  auto transform_to_geo_tf = [](const tf2::Transform& transform) {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = transform.getOrigin().x();
    msg.translation.y = transform.getOrigin().y();
    msg.translation.z = transform.getOrigin().z();
    msg.rotation.x = transform.getRotation().x();
    msg.rotation.y = transform.getRotation().y();
    msg.rotation.z = transform.getRotation().z();
    msg.rotation.w = transform.getRotation().w();
    return msg;
  };

  // decide which foot is which
  if (ik_goals.is_left_support_foot) {
    left_foot_goal = transform_to_geo_tf(trunk_to_support_foot_goal);
    right_foot_goal = transform_to_geo_tf(trunk_to_flying_foot_goal);
  } else {
    right_foot_goal = transform_to_geo_tf(trunk_to_support_foot_goal);
    left_foot_goal = transform_to_geo_tf(trunk_to_flying_foot_goal);
  }

  auto transform_to_geo_pose = [](const geometry_msgs::msg::Transform& transform) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = transform.translation.x;
    msg.position.y = transform.translation.y;
    msg.position.z = transform.translation.z;
    msg.orientation.x = transform.rotation.x;
    msg.orientation.y = transform.rotation.y;
    msg.orientation.z = transform.rotation.z;
    msg.orientation.w = transform.rotation.w;
    return msg;
  };

  left_foot_goal_ = transform_to_geo_pose(left_foot_goal);
  right_foot_goal_ = transform_to_geo_pose(right_foot_goal);

  bitbots_splines::JointGoals result;

  // call IK
  Eigen::Isometry3d right_iso = tf2::transformToEigen(right_foot_goal);
  Mat4 T_right = right_iso.matrix();
  auto result_right = calculate_ik_right_leg(T_right);

  for (const auto& joint : getRightLegJointNames()) {
    result.first.push_back(joint);
    result.second.push_back(result_right[joint.substr(1)]);  // remove the "R" from the joint name to get the corresponding left joint
  }

  Eigen::Isometry3d left_iso = tf2::transformToEigen(left_foot_goal);
  Mat4 T_left = left_iso.matrix();
  auto result_left = calculate_ik_left_leg(T_left);

  for (const auto& joint : getLeftLegJointNames()) {
    result.first.push_back(joint);
    result.second.push_back(result_left[joint.substr(1)]);  // remove the "R" from the joint name to get the corresponding right joint
  }

  return result;
}

void WalkIK::reset() {
}

void WalkIK::setConfig(walking::Params::Node::Ik config) { config_ = config; }

const std::vector<std::string> WalkIK::getLeftLegJointNames() {
  return {
    "LHipYaw", "LHipRoll", "LHipPitch",
    "LKnee",
    "LAnklePitch", "LAnkleRoll"
  };
}

const std::vector<std::string> WalkIK::getRightLegJointNames() {
  return {
    "RHipYaw", "RHipRoll", "RHipPitch",
    "RKnee",
    "RAnklePitch", "RAnkleRoll"
  };
}

const geometry_msgs::msg::Pose WalkIK::get_right_goal() {
  return right_foot_goal_;
}

const geometry_msgs::msg::Pose WalkIK::get_left_goal() {
  return left_foot_goal_;
}

}  // namespace bitbots_quintic_walk
