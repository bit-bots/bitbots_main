//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_MOTIONMODEL_H
#define BITBOTS_LOCALIZATION_MOTIONMODEL_H

#include <memory>
#include <cstdlib>

#include <particle_filter/MovementModel.h>
#include <bitbots_localization/RobotState.h>
#include <geometry_msgs/Vector3.h>
#include <particle_filter/CRandomNumberGenerator.h>
#include <bitbots_localization/tools.h>

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
  RobotMotionModel(const particle_filter::CRandomNumberGenerator &random_number_generator,
                  double diffuse_xStdDev,
                  double diffuse_yStdDev,
                  double diffuse_tStdDev,
                  double diffuse_multiplicator,
                  Eigen::Matrix<double, 3, 2> drift_cov);

  /**
   * The drift method is empty in this example.
   * @param state Pointer to the state that has to be manipulated.
   */
  void drift(RobotState &state, geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular) const override;

  /**
   * The diffusion consists of a very small gaussian jitter on the
   * state's variable.
   * @param state Pointer to the state that has to be manipulated.
   */
  void diffuse(RobotState &state) const override;

 protected:

 private:

  // The random number generator
  particle_filter::CRandomNumberGenerator random_number_generator_;

  // standard deviations and multiplicator for the diffuse step
  double diffuse_xStdDev_, diffuse_yStdDev_, diffuse_tStdDev_, diffuse_multiplicator_;
  Eigen::Matrix<double, 3, 2>  drift_cov_;

  double sample(double b) const;

};

#endif //BITBOTS_LOCALIZATION_MOTIONMODEL_H
