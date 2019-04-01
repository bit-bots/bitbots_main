#include <bitbots_world_model/MovementModels.h>

LocalObstacleMovementModel::LocalObstacleMovementModel(
        particle_filter::CRandomNumberGenerator& random_number_generator,
        double xStdDev,
        double yStdDev,
        double multiplicator) :
        particle_filter::MovementModel<PositionStateW>(),
        random_number_generator_(random_number_generator),
        xStdDev_(xStdDev),
        yStdDev_(yStdDev),
        multiplicator_(multiplicator) {}

LocalObstacleMovementModel::~LocalObstacleMovementModel() {}

void LocalObstacleMovementModel::drift(PositionStateW& state,
        geometry_msgs::Vector3 linear,
        geometry_msgs::Vector3 angular) const {
    // apply linear movement
    state.setXPos(state.getXPos() - linear.x);
    state.setYPos(state.getYPos() - linear.y);
    // calculate polar coordinates of state
    double r = std::sqrt(
            std::pow(state.getXPos(), 2) +
            std::pow(state.getYPos(),
                    2));  // as this is done after the application of linear
                          // movement, the "robot" first walks linearly and
                          // turns then in this modelation.
    double d = std::atan(state.getXPos() /
                         state.getYPos());  // make sure, you did not mix up x
                                            // and y!!! The result is a radian
    // make sure not to mix up radian and degree!
    // apply angular movement on polar coordinate
    d -= angular.z;  // find the right element of the vector
    // calculate cartesian coordinate of state and write it back into it
    state.setXPos(r * std::cos(d));
    state.setYPos(r * std::sin(d));
    // this can be done more sophisticated by combining the movement (thr robot
    // walks a curve and does not move first and turn then) this can be
    // irrelevant due to high update rates.
    // TODO: apply noise to the measurements!
}

void LocalObstacleMovementModel::diffuse(PositionStateW& state) const {
    state.setXPos(
            state.getXPos() +
            random_number_generator_.getGaussian(xStdDev_) * multiplicator_);
    state.setYPos(
            state.getYPos() +
            random_number_generator_.getGaussian(yStdDev_) * multiplicator_);
}

LocalRobotMovementModel::LocalRobotMovementModel(
        particle_filter::CRandomNumberGenerator& random_number_generator,
        double xStdDev,
        double yStdDev,
        double multiplicator) :
        particle_filter::MovementModel<PositionState>(),
        random_number_generator_(random_number_generator),
        xStdDev_(xStdDev),
        yStdDev_(yStdDev),
        multiplicator_(multiplicator) {}

LocalRobotMovementModel::~LocalRobotMovementModel() {}

void LocalRobotMovementModel::drift(PositionState& state,
        geometry_msgs::Vector3 linear,
        geometry_msgs::Vector3 angular) const {
    // apply linear movement
    state.setXPos(state.getXPos() - linear.x);
    state.setYPos(state.getYPos() - linear.y);
    // calculate polar coordinates of state
    double r = std::sqrt(
            std::pow(state.getXPos(), 2) +
            std::pow(state.getYPos(),
                    2));  // as this is done after the application of linear
                          // movement, the "robot" first walks linearly and
                          // turns then in this modelation.
    double d = std::atan(state.getXPos() /
                         state.getYPos());  // make sure, you did not mix up x
                                            // and y!!! The result is a radian
    // make sure not to mix up radian and degree!
    // apply angular movement on polar coordinate
    d -= angular.z;  // find the right element of the vector
    // calculate cartesian coordinate of state and write it back into it
    state.setXPos(r * std::cos(d));
    state.setYPos(r * std::sin(d));
    // this can be done more sophisticated by combining the movement (thr robot
    // walks a curve and does not move first and turn then) this can be
    // irrelevant due to high update rates.
    // TODO: apply noise to the measurements!
}

void LocalRobotMovementModel::diffuse(PositionState& state) const {
    state.setXPos(
            state.getXPos() +
            random_number_generator_.getGaussian(xStdDev_) * multiplicator_);
    state.setYPos(
            state.getYPos() +
            random_number_generator_.getGaussian(yStdDev_) * multiplicator_);
}
