//
// Created by judith on 09.03.19.
//

#include "../include/bitbots_localization/MotionModel.h"

RobotMotionModel::RobotMotionModel(const particle_filter::CRandomNumberGenerator &random_number_generator,
                                   double xStdDev,
                                   double yStdDev,
                                   double tStdDev,
                                   double multiplicator) : particle_filter::MovementModel<RobotState>(),
                                                           random_number_generator_(random_number_generator),
                                                           xStdDev_(xStdDev),
                                                           yStdDev_(yStdDev),
                                                           tStdDev_(tStdDev),
                                                           multiplicator_(multiplicator) {

}

void RobotMotionModel::drift(RobotState &state,
                             geometry_msgs::Vector3 linear_movement,
                             geometry_msgs::Vector3 rotational_movement) const {

  // Convert cartesian coordinates to polarcoordinates with an orientation
  auto [polar_rot, polar_dist] = cartesianToPolar(linear_movement.x, linear_movement.y);
  double orientation = rotational_movement.z - polar_rot;

  // Apply sample drift for odom data
  double deltaRot1_with_offset = polar_rot - sample(0.001 * abs(polar_rot) + 0.001 * polar_dist);
  double deltaTrans_with_offset = polar_dist - sample(0.001 * polar_dist + 0.001 * (abs(polar_rot) + abs(orientation)));
  double deltaRot2_with_offset = orientation - sample(0.001 * abs(orientation) + 0.001 * polar_dist);

  // Convert polar coordinates with offset back to cartesian ones
  auto [cartesian_with_offset_x, cartesian_with_offset_y] = polarToCartesian(
    state.getTheta() + deltaRot1_with_offset,
    deltaTrans_with_offset);

  // Apply new values onto state
  state.setXPos(state.getXPos() + cartesian_with_offset_x);
  state.setYPos(state.getYPos() + cartesian_with_offset_y);
  double theta = state.getTheta() + deltaRot1_with_offset + deltaRot2_with_offset;

  if (theta > M_PI) {
    theta = -M_PI + (theta - M_PI);
  } else if (theta < -M_PI) {
    theta = M_PI + (theta + M_PI);
  }
  state.setTheta(theta);

}

void RobotMotionModel::diffuse(RobotState &state) const {

  state.setXPos(state.getXPos() + sample(xStdDev_) * multiplicator_);
  state.setYPos(state.getYPos() + sample(yStdDev_) * multiplicator_);
  double theta = state.getTheta() + sample(tStdDev_) * multiplicator_;
  if (theta > M_PI) {
    theta = -M_PI + (theta - M_PI);
  } else if (theta < -M_PI) {
    theta = M_PI + (theta + M_PI);
  }
  state.setTheta(theta);
}

double RobotMotionModel::sample(double b) const {

  return random_number_generator_.getGaussian(b);
}
