//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H
#define BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H

#include <bitbots_localization/RobotState.hpp>
#include <particle_filter/CRandomNumberGenerator.hpp>
#include <particle_filter/StateDistribution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace bitbots_localization {

class RobotStateDistribution : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistribution(particle_filter::CRandomNumberGenerator& random_number_generator,
                         std::pair<double, double> initial_robot_pose, std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  std::pair<double, double> initial_robot_pose_;
};

class RobotStateDistributionStartLeft : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionStartLeft(particle_filter::CRandomNumberGenerator& random_number_generator,
                                  std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  std::pair<double, double> field_size;
};

class RobotStateDistributionOwnSideline : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionOwnSideline(particle_filter::CRandomNumberGenerator& random_number_generator,
                                    std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double field_x, field_y;
};

class RobotStateDistributionOpponentHalf : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionOpponentHalf(particle_filter::CRandomNumberGenerator& random_number_generator,
                                     std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
};

class RobotStateDistributionOwnHalf : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionOwnHalf(particle_filter::CRandomNumberGenerator& random_number_generator,
                                std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
};

class RobotStateDistributionPosition : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionPosition(particle_filter::CRandomNumberGenerator& random_number_generator, double x, double y);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double x_;
  double y_;
};

class RobotStateDistributionPose : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionPose(particle_filter::CRandomNumberGenerator& random_number_generator, double x, double y,
                             double t);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double x_;
  double y_;
  double t_;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H
