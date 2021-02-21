#include "bitbots_dynamic_kick/stabilizer.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer() {
  ros::NodeHandle pid_x_nh = ros::NodeHandle("dynamic_kick/pid_x");
  ros::NodeHandle pid_y_nh = ros::NodeHandle("dynamic_kick/pid_y");
  pid_x_.init(pid_x_nh, false);
  pid_y_.init(pid_y_nh, false);
  reset();
}

void Stabilizer::reset() {
  pid_x_.reset();
  pid_y_.reset();
}

KickPositions Stabilizer::stabilize(const KickPositions &positions, const ros::Duration &dt) {
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

    double x_correction = pid_x_.computeCommand(cop_x_error, dt);
    double y_correction = pid_y_.computeCommand(cop_y_error, dt);

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
