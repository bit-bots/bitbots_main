//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/StateDistribution.hpp>

namespace bitbots_localization {

RobotStateDistributionStartLeft::RobotStateDistributionStartLeft(
    particle_filter::CRandomNumberGenerator &random_number_generator, std::pair<double, double> field_size) {
  field_size = field_size;
}

const RobotState RobotStateDistributionStartLeft::draw() const {
  if (random_number_generator_.getUniform(0, 1) > 0.5) {
    return (RobotState(random_number_generator_.getUniform(field_size.first / 2, 0.0),
                       random_number_generator_.getGaussian(0.1) - field_size.second / 2 - 0.1,
                       random_number_generator_.getGaussian(0.2) - 1.57));
  } else {
    return (RobotState(random_number_generator_.getUniform(field_size.first / 2, 0.0),
                       random_number_generator_.getGaussian(0.1) + field_size.second / 2 + 0.1,
                       random_number_generator_.getGaussian(0.2) + 1.57));
  }
}

RobotStateDistributionOwnSideline::RobotStateDistributionOwnSideline(
    particle_filter::CRandomNumberGenerator &random_number_generator, std::pair<double, double> field_size) {
  field_x = field_size.first;
  field_y = field_size.second;
}

const RobotState RobotStateDistributionOwnSideline::draw() const {
  if (random_number_generator_.getUniform(0, 1) > 0.5) {
    return (RobotState(random_number_generator_.getUniform(-field_x / 2, 0.0),
                       random_number_generator_.getGaussian(0.1) - field_y / 2,
                       random_number_generator_.getGaussian(0.2) + 1.57));
  } else {
    return (RobotState(random_number_generator_.getUniform(-field_x / 2, 0.0),
                       random_number_generator_.getGaussian(0.1) + field_y / 2,
                       random_number_generator_.getGaussian(0.2) - 1.57));
  }
}

RobotStateDistributionOpponentHalf::RobotStateDistributionOpponentHalf(
    particle_filter::CRandomNumberGenerator &random_number_generator, std::pair<double, double> field_size)
    : random_number_generator_(random_number_generator) {
  // only own half
  min_x_ = (field_size.first / 2.0) + 0.5;
  min_y_ = (-field_size.second / 2.0) - 0.5;
  max_x_ = 0 - 0.5;
  max_y_ = (field_size.second / 2.0) + 0.5;
}

const RobotState RobotStateDistributionOpponentHalf::draw() const {
  return (RobotState(random_number_generator_.getUniform(min_x_, max_x_),
                     random_number_generator_.getUniform(min_y_, max_y_),
                     random_number_generator_.getUniform(-M_PI, M_PI)));
}

RobotStateDistributionOwnHalf::RobotStateDistributionOwnHalf(
    particle_filter::CRandomNumberGenerator &random_number_generator, std::pair<double, double> field_size)
    : random_number_generator_(random_number_generator) {
  // only own half
  min_x_ = -field_size.first / 2.0;
  min_y_ = -field_size.second / 2.0;
  max_x_ = 0;
  max_y_ = field_size.second / 2.0;
}

const RobotState RobotStateDistributionOwnHalf::draw() const {
  return (RobotState(random_number_generator_.getUniform(min_x_, max_x_),
                     random_number_generator_.getUniform(min_y_, max_y_),
                     random_number_generator_.getUniform(-M_PI, M_PI)));
}

RobotStateDistributionPosition::RobotStateDistributionPosition(
    particle_filter::CRandomNumberGenerator &random_number_generator, double x, double y) {
  x_ = x;
  y_ = y;
}

const RobotState RobotStateDistributionPosition::draw() const {
  return (RobotState(random_number_generator_.getGaussian(0.4) + x_, random_number_generator_.getGaussian(0.4) + y_,
                     random_number_generator_.getUniform(-M_PI, M_PI)));
}

RobotStateDistributionPose::RobotStateDistributionPose(particle_filter::CRandomNumberGenerator &random_number_generator,
                                                       double x, double y, double t) {
  x_ = x;
  y_ = y;
  t_ = t;
}

const RobotState RobotStateDistributionPose::draw() const {
  return RobotState(random_number_generator_.getGaussian(0.1) + x_, random_number_generator_.getGaussian(0.1) + y_,
                    random_number_generator_.getGaussian(0.1) + t_);
}
}  // namespace bitbots_localization
