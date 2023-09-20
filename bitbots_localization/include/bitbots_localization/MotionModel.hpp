//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_MOTIONMODEL_H
#define BITBOTS_LOCALIZATION_MOTIONMODEL_H

#include <particle_filter/CRandomNumberGenerator.h>
#include <particle_filter/MovementModel.h>

#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/tools.hpp>
#include <cstdlib>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

namespace bitbots_localization {
/**
 * @class MyMovementModel
 *
 * @brief Test class for ParticleFilter.
 *
 * This simple movement model only adds a small jitter in diffuse() and does
 * nothing in drift().
 * @author Stephan Wirth
 */
class RobotMotionModel : public particle_filter::MovementModel<RobotState> {
 public:
  /**
   * empty

   */
  RobotMotionModel(const particle_filter::CRandomNumberGenerator &random_number_generator, double diffuse_xStdDev,
                   double diffuse_yStdDev, double diffuse_tStdDev, double diffuse_multiplicator,
                   Eigen::Matrix<double, 3, 2> drift_cov);

  /**
   * @param state Pointer to the state that has to be manipulated.
   * @param linear Linear movement relative to the base footprint in cartesian space
   * @param angular Anular movement of of the robot in its z-axis, therefore only the z axis needs to be set
   */
  void drift(RobotState &state, geometry_msgs::msg::Vector3 linear, geometry_msgs::msg::Vector3 angular) const override;

  /**
   * The diffusion consists of a very small gaussian jitter on the
   * state's variable.
   * @param state Pointer to the state that has to be manipulated.
   */
  void diffuse(RobotState &state) const override;

  double diffuse_multiplicator_;

 protected:
 private:
  // The random number generator
  particle_filter::CRandomNumberGenerator random_number_generator_;

  // standard deviations and multiplicator for the diffuse step
  double diffuse_xStdDev_, diffuse_yStdDev_, diffuse_tStdDev_;
  Eigen::Matrix<double, 3, 2> drift_cov_;

  double sample(double b) const;
};
};      // namespace bitbots_localization
#endif  // BITBOTS_LOCALIZATION_MOTIONMODEL_H
