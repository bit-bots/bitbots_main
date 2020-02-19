//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H
#define BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H

#include <utility>

#include <particle_filter/CRandomNumberGenerator.h>
#include <particle_filter/StateDistribution.h>
#include <bitbots_localization/RobotState.h>
#include <ros/ros.h>

class RobotStateDistribution : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistribution(particle_filter::CRandomNumberGenerator &random_number_generator,
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
  RobotStateDistributionStartLeft(particle_filter::CRandomNumberGenerator &random_number_generator,
                                  std::pair<double, double> initial_robot_pose1,
                                  double initial_theta1,
                                  std::pair<double, double> initial_robot_pose2,
                                  double initial_theta2,
                                  std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  std::pair<double, double> initial_robot_pose_1;
  std::pair<double, double> initial_robot_pose_2;
  double initial_theta_1;
  double initial_theta_2;

};

class RobotStateDistributionStartRight : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionStartRight(particle_filter::CRandomNumberGenerator &random_number_generator,
                                   std::pair<double, double> initial_robot_pose1,
                                   double initial_theta1,
                                   std::pair<double, double> initial_robot_pose2,
                                   double initial_theta2,
                                   std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  std::pair<double, double> initial_robot_pose_1;
  std::pair<double, double> initial_robot_pose_2;
  double initial_theta_1;
  double initial_theta_2;

};

class RobotStateDistributionLeftHalf : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionLeftHalf(particle_filter::CRandomNumberGenerator &random_number_generator,
                                 std::pair<double, double> field_size);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;

};

class RobotStateDistributionRightHalf : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionRightHalf(particle_filter::CRandomNumberGenerator &random_number_generator,
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
  RobotStateDistributionPosition(particle_filter::CRandomNumberGenerator &random_number_generator,
                                 double x,
                                 double y);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double x_;
  double y_;

};

class RobotStateDistributionPose : public particle_filter::StateDistribution<RobotState> {
 public:
  RobotStateDistributionPose(particle_filter::CRandomNumberGenerator &random_number_generator,
                             double x,
                             double y,
                             double t);

  const RobotState draw() const override;

 private:
  particle_filter::CRandomNumberGenerator random_number_generator_;
  double x_;
  double y_;
  double t_;

};

#endif //BITBOTS_LOCALIZATION_STATEDISTRIBUTION_H
