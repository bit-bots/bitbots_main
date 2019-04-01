#ifndef MOVEMENTMODELS
#define MOVEMENTMODELS

#include <memory>

#include <particle_filter/MovementModel.h>
#include <bitbots_world_model/ObstacleStates.h>
#include <geometry_msgs/Vector3.h>
// #include <libPF/CRandomNumberGenerator.h>

/**
 * @class MyMovementModel
 *
 * @brief Test class for ParticleFilter.
 *
 * This simple movement model only adds a small jitter in diffuse() and does
 * nothing in drift().
 * @author Stephan Wirth
 */
class LocalObstacleMovementModel
        : public particle_filter::MovementModel<PositionStateW> {
public:
    /**
     * empty
     */
    LocalObstacleMovementModel(
            particle_filter::CRandomNumberGenerator& random_number_generator,
            double xStdDev,
            double yStdDev,
            double multiplicator);

    /**
     * empty
     */
    ~LocalObstacleMovementModel();

    /**
     * The drift method is empty in this example.
     * @param state Pointer to the state that has to be manipulated.
     */
    void drift(PositionStateW& state,
            geometry_msgs::Vector3 linear,
            geometry_msgs::Vector3 angular) const;

    /**
     * The diffusion consists of a very small gaussian jitter on the
     * state's variable.
     * @param state Pointer to the state that has to be manipulated.
     */
    void diffuse(PositionStateW& state) const;

protected:
private:
    // The random number generator
    particle_filter::CRandomNumberGenerator random_number_generator_;

    // standard deviations and multiplicator for the diffuse step
    double xStdDev_, yStdDev_, multiplicator_;
};

class LocalRobotMovementModel
        : public particle_filter::MovementModel<PositionState> {
public:
    /**
     * empty
     */
    LocalRobotMovementModel(
            particle_filter::CRandomNumberGenerator& random_number_generator,
            double xStdDev,
            double yStdDev,
            double multiplicator);

    /**
     * empty
     */
    ~LocalRobotMovementModel();

    /**
     * The drift method is empty in this example.
     * @param state Pointer to the state that has to be manipulated.
     */
    void drift(PositionState& state,
            geometry_msgs::Vector3 linear,
            geometry_msgs::Vector3 angular) const;

    /**
     * The diffusion consists of a very small gaussian jitter on the
     * state's variable.
     * @param state Pointer to the state that has to be manipulated.
     */
    void diffuse(PositionState& state) const;

protected:
private:
    // The random number generator
    particle_filter::CRandomNumberGenerator random_number_generator_;

    // standard deviations and multiplicator for the diffuse step
    double xStdDev_, yStdDev_, multiplicator_;
};

#endif
