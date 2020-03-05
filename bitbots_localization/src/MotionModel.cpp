//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/MotionModel.h>

RobotMotionModel::RobotMotionModel(
  const particle_filter::CRandomNumberGenerator &random_number_generator,
  double diffuse_xStdDev,
  double diffuse_yStdDev,
  double diffuse_tStdDev,
  double diffuse_multiplicator,
  std::array <std::array<double, 2>, 3> drift_cov
  ) : particle_filter::MovementModel<RobotState>(),
      random_number_generator_(random_number_generator),
      diffuse_xStdDev_(diffuse_xStdDev),
      diffuse_yStdDev_(diffuse_yStdDev),
      diffuse_tStdDev_(diffuse_tStdDev),
      diffuse_multiplicator_(diffuse_multiplicator),
      drift_cov_(drift_cov) {

}

void RobotMotionModel::drift(RobotState &state,
                             geometry_msgs::Vector3 linear_movement,
                             geometry_msgs::Vector3 rotational_movement) const {

  // Convert cartesian coordinates to polarcoordinates with an orientation
  auto [polar_rot, polar_dist] = cartesianToPolar(linear_movement.x, linear_movement.y);
  double orientation = rotational_movement.z;

  // Apply sample drift for odom data
  //no need for abs polar distance, because its positive every time
  double polar_rot_with_drift   = polar_rot   - sample(drift_cov_[0][0] * polar_dist + drift_cov_[0][1] * abs(orientation));
  double polar_dist_with_drift  = polar_dist  - sample(drift_cov_[1][0] * polar_dist + drift_cov_[1][0] * abs(orientation));
  double orientation_with_drift = orientation - sample(drift_cov_[2][0] * polar_dist + drift_cov_[2][0] * abs(orientation));

  // Convert polar coordinates with offset back to cartesian ones, while transforming it into the local frame of each particle
  auto [cartesian_with_offset_x, cartesian_with_offset_y] = polarToCartesian(
    state.getTheta() + polar_rot_with_drift, polar_dist_with_drift);

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

  state.setXPos(state.getXPos() + sample(diffuse_xStdDev_) * diffuse_multiplicator_);
  state.setYPos(state.getYPos() + sample(diffuse_yStdDev_) * diffuse_multiplicator_);
  double theta = state.getTheta() + sample(diffuse_tStdDev_) * diffuse_multiplicator_;
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
