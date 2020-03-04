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
  double orientation = rotational_movement.z;

  float cov[3][2] {
    // Standard dev of applied drift related to
    // distance, rotation
      {0.0000  , 0.0000},  // Values affecting walking direction
      {0.0000  , 0.0000},  // Values affecting walking distance
      {0.0000  , 0.0000}}; // Values affecting orientation

  // Apply sample drift for odom data
  //no need for abs polar distance, because its positive every time
  double polar_rot_with_drift   = polar_rot   - sample(cov[0][0] * polar_dist + cov[0][1] * abs(orientation));
  double polar_dist_with_drift  = polar_dist  - sample(cov[1][0] * polar_dist + cov[1][0] * abs(orientation));
  double orientation_with_drift = orientation - sample(cov[2][0] * polar_dist + cov[2][0] * abs(orientation));

  // Convert polar coordinates with offset back to cartesian ones
  auto [cartesian_with_offset_x, cartesian_with_offset_y] = polarToCartesian(
    polar_rot_with_drift, polar_dist_with_drift);

  // Apply new values onto state
  state.setXPos(state.getXPos() + cartesian_with_offset_x);
  state.setYPos(state.getYPos() + cartesian_with_offset_y);
  double theta = state.getTheta() + orientation_with_drift;

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
