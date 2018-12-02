#include <bitbots_world_model/MovementModels.h>

LocalObstacleMovementModel::LocalObstacleMovementModel(libPF::CRandomNumberGenerator& random_number_generator, double xStdDev, double yStdDev, double multiplicator) : libPF::MovementModel<PositionStateW>(),
                                                                                                                                                                  random_number_generator_(random_number_generator),
                                                                                                                                                                  xStdDev_(xStdDev),
                                                                                                                                                                  yStdDev_(yStdDev),
                                                                                                                                                                  multiplicator_(multiplicator){
}

LocalObstacleMovementModel::~LocalObstacleMovementModel() {
}

void LocalObstacleMovementModel::drift(PositionStateW& state, geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular) const {
    // apply linear movement
    // calculate polar coordinates of state
    // make sure not to mix up radian and degree!
    // apply angular movement on polar coordinate
    // calculate cartesian coordinate of state
    // write into state
    // this can be done more sophisticated by combining the movement (thr robot walks a curve and does not move first and turn then) this can be irrelevant due to high update rates.
}

void LocalObstacleMovementModel::diffuse(PositionStateW& state, double /*dt*/) const {
    state.setXPos(state.getXPos() + random_number_generator_.getGaussian(xStdDev_) * multiplicator_);
    state.setYPos(state.getYPos() + random_number_generator_.getGaussian(yStdDev_) * multiplicator_);
}

LocalRobotMovementModel::LocalRobotMovementModel(libPF::CRandomNumberGenerator& random_number_generator, double xStdDev, double yStdDev, double multiplicator) : libPF::MovementModel<PositionState>(),
                                                                                                                                                                  random_number_generator_(random_number_generator),
                                                                                                                                                                  xStdDev_(xStdDev),
                                                                                                                                                                  yStdDev_(yStdDev),
                                                                                                                                                                  multiplicator_(multiplicator){
}

LocalRobotMovementModel::~LocalRobotMovementModel() {
}

void LocalRobotMovementModel::drift(PositionState& state, geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular) const {
    // apply linear movement
    // calculate polar coordinates of state
    // make sure not to mix up radian and degree!
    // apply angular movement on polar coordinate
    // calculate cartesian coordinate of state
    // write into state
    // this can be done more sophisticated by combining the movement (thr robot walks a curve and does not move first and turn then) this can be irrelevant due to high update rates.
}

void LocalRobotMovementModel::diffuse(PositionState& state, double /*dt*/) const {
    state.setXPos(state.getXPos() + random_number_generator_.getGaussian(xStdDev_) * multiplicator_);
    state.setYPos(state.getYPos() + random_number_generator_.getGaussian(yStdDev_) * multiplicator_);
}

