#include "bitbots_dynamic_kick/stabilizer.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer(std::string ns) {
  pitch_node_ = rclcpp::Node::make_shared(ns + "pid_trunk_fused_pitch");
  roll_node_ = rclcpp::Node::make_shared(ns + "pid_trunk_fused_roll");
  pitch_node_->get_logger().set_level(rclcpp::Logger::Level::Warn);
  roll_node_->get_logger().set_level(rclcpp::Logger::Level::Warn);

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

void Stabilizer::reset() {
  pid_trunk_fused_pitch_->reset();
  pid_trunk_fused_roll_->reset();
}

KickPositions Stabilizer::stabilize(const KickPositions &positions, const rclcpp::Duration &dt) {
  KickPositions stabilized_positions = positions;
  if (positions.cop_support_point && use_cop_) {
    /* calculate stabilizing target from center of pressure
     * the cop is in corresponding sole frame
     * optimal stabilizing would be centered above sole center */
    double cop_x, cop_y, cop_x_error, cop_y_error;
    if (positions.is_left_kick) {
      cop_x = cop_right.x;
      cop_y = cop_right.y;
    } else {
      cop_x = cop_left.x;
      cop_y = cop_left.y;
    }
    cop_x_error = cop_x - positions.trunk_pose.translation().x();
    cop_y_error = cop_y - positions.trunk_pose.translation().y();

    double x_correction = pid_trunk_fused_roll_->computeCommand(cop_x_error, dt);
    double y_correction = pid_trunk_fused_pitch_->computeCommand(cop_y_error, dt);

    stabilized_positions.trunk_pose.translation().x() += x_correction;
    stabilized_positions.trunk_pose.translation().y() += y_correction;
  }
  return stabilized_positions;
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

}
