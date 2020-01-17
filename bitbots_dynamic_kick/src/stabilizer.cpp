#include <utility>

#include "bitbots_dynamic_kick/stabilizer.h"
#include "bitbots_splines/dynamic_balancing_goal.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer() {
  reset();
}

void Stabilizer::reset() {
  cop_x_error_sum_ = 0.0;
  cop_y_error_sum_ = 0.0;
}

KickPositions Stabilizer::stabilize(const KickPositions &positions, const ros::Duration &dt) {
  KickPositions stabilized_positions = positions;
  if (positions.cop_support_point && use_cop_) {
    // TODO use existing PID library
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
    cop_x_error = cop_x - positions.trunk_pose.getOrigin().getX();
    cop_y_error = cop_y - positions.trunk_pose.getOrigin().getY();
    cop_x_error_sum_ += cop_x_error;
    cop_y_error_sum_ += cop_y_error;
    double x = positions.trunk_pose.getOrigin().getX() - cop_x * p_x_factor_ - i_x_factor_ * cop_x_error_sum_
        - d_x_factor_ * (cop_x_error - cop_x_error_);
    double y = positions.trunk_pose.getOrigin().getY() - cop_y * p_y_factor_ - i_y_factor_ * cop_y_error_sum_
        - d_y_factor_ * (cop_y_error - cop_y_error_);
    cop_x_error_ = cop_x_error;
    cop_y_error_ = cop_y_error;
    /* Do not use control for height and rotation */
    stabilized_positions.trunk_pose.setOrigin({x, y, positions.trunk_pose.getOrigin().getZ()});
  }

  return stabilized_positions;
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setFlyingWeight(double weight) {
  flying_weight_ = weight;
}

void Stabilizer::setTrunkWeight(double weight) {
  trunk_weight_ = weight;
}

void Stabilizer::setPFactor(double factor_x, double factor_y) {
  p_x_factor_ = factor_x;
  p_y_factor_ = factor_y;
}

void Stabilizer::setIFactor(double factor_x, double factor_y) {
  i_x_factor_ = factor_x;
  i_y_factor_ = factor_y;
}

void Stabilizer::setDFactor(double factor_x, double factor_y) {
  d_x_factor_ = factor_x;
  d_y_factor_ = factor_y;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

}
