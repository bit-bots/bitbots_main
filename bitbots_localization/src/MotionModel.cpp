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
                             geometry_msgs::Vector3 movement,
                             geometry_msgs::Vector3 movement2) const {
  // sample_motion_model_odometry
  double deltaRot1New = movement.x - sample(0.001 * abs(movement.x) + 0.001 * movement.y);
  double deltaTransNew = movement.y - sample(0.001 * movement.y + 0.001 * (abs(movement.x) + abs(movement.z)));
  double deltaRot2New = movement.z - sample(0.001 * abs(movement.z) + 0.001 * movement.y);

  state.setXPos(state.getXPos() + deltaTransNew * cos(state.getTheta() + deltaRot1New));
  state.setYPos(state.getYPos() + deltaTransNew * sin(state.getTheta() + deltaRot1New));
  double theta = state.getTheta() + deltaRot1New + deltaRot2New;
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
