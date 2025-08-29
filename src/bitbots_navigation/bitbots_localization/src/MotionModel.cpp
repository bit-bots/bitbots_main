//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/MotionModel.hpp>

namespace bitbots_localization {

RobotMotionModel::RobotMotionModel(const particle_filter::CRandomNumberGenerator &random_number_generator,
                                   double diffuse_xStdDev, double diffuse_yStdDev, double diffuse_tStdDev,
                                   double diffuse_multiplier, Eigen::Matrix<double, 3, 2> drift_cov)
    : particle_filter::MovementModel<RobotState>(),
      diffuse_multiplier_(diffuse_multiplier),
      random_number_generator_(random_number_generator),
      diffuse_xStdDev_(diffuse_xStdDev),
      diffuse_yStdDev_(diffuse_yStdDev),
      diffuse_tStdDev_(diffuse_tStdDev),
      drift_cov_(drift_cov) {}

void RobotMotionModel::drift(RobotState &state, geometry_msgs::msg::Vector3 linear_movement,
                             geometry_msgs::msg::Vector3 rotational_movement) const {
  // Convert cartesian coordinates to polarcoordinates with an orientation
  auto [polar_rot, polar_dist] = cartesianToPolar(linear_movement.x, linear_movement.y);
  // get the minimal absolute
  double orientation = signedAngle(rotational_movement.z);
  // Apply sample drift for odom data
  // no need for abs polar distance, because its positive every time
  double polar_rot_with_drift = polar_rot - sample(drift_cov_(0, 0) * polar_dist + drift_cov_(0, 1) * abs(orientation));
  double polar_dist_with_drift =
      polar_dist - sample(drift_cov_(1, 0) * polar_dist + drift_cov_(1, 1) * abs(orientation));
  double orientation_with_drift =
      orientation - sample(drift_cov_(2, 0) * polar_dist + drift_cov_(2, 1) * abs(orientation));

  // Convert polar coordinates with offset back to cartesian ones, while transforming it into the local frame of each
  // particle
  auto [cartesian_with_offset_x, cartesian_with_offset_y] =
      polarToCartesian(state.getTheta() + polar_rot_with_drift, polar_dist_with_drift);

  // Apply new values onto state
  state.setXPos(state.getXPos() + cartesian_with_offset_x);
  state.setYPos(state.getYPos() + cartesian_with_offset_y);
  double theta = state.getTheta() + orientation_with_drift;

  state.setTheta(theta);
}

void RobotMotionModel::diffuse(RobotState &state) const {
  state.setXPos(state.getXPos() + sample(diffuse_xStdDev_) * diffuse_multiplier_);
  state.setYPos(state.getYPos() + sample(diffuse_yStdDev_) * diffuse_multiplier_);
  double theta = state.getTheta() + sample(diffuse_tStdDev_) * diffuse_multiplier_;
  state.setTheta(theta);
}

double RobotMotionModel::sample(double b) const { return random_number_generator_.getGaussian(b); }
}  // namespace bitbots_localization
