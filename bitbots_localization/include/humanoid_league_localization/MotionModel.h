//
// Created by judith on 09.03.19.
//

#ifndef HUMANOID_LEAGUE_LOCALIZATION_MOTIONMODEL_H
#define HUMANOID_LEAGUE_LOCALIZATION_MOTIONMODEL_H

#include <memory>
#include <stdlib.h>

#include <particle_filter/MovementModel.h>
#include <humanoid_league_localization/RobotState.h>
#include <geometry_msgs/Vector3.h>
#include <particle_filter/CRandomNumberGenerator.h>
#include <humanoid_league_localization/tools.h>

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
    RobotMotionModel(particle_filter::CRandomNumberGenerator& random_number_generator, double xStdDev, double yStdDev, double tStdDev, double multiplicator);

    /**
     * empty
     */
    ~RobotMotionModel();

    /**
     * The drift method is empty in this example.
     * @param state Pointer to the state that has to be manipulated.
     */
    void drift(RobotState& state, geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular) const;


    /**
     * The diffusion consists of a very small gaussian jitter on the
     * state's variable.
     * @param state Pointer to the state that has to be manipulated.
     */
    void diffuse(RobotState& state) const;

protected:

private:

    // The random number generator
    particle_filter::CRandomNumberGenerator random_number_generator_;

    // standard deviations and multiplicator for the diffuse step
    double xStdDev_, yStdDev_, tStdDev_,  multiplicator_;

    double sample(double b) const;

};


#endif //HUMANOID_LEAGUE_LOCALIZATION_MOTIONMODEL_H
