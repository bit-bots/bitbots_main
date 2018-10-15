#include <bitbots_world_model/MovementModels.h>

LocalObstacleMovementModel::LocalObstacleMovementModel(libPF::CRandomNumberGenerator& random_number_generator, double xStdDev, double yStdDev, double multiplicator) : libPF::MovementModel<ObstacleStateW>(),
                                                                                                                                                                  random_number_generator_(random_number_generator),
                                                                                                                                                                  xStdDev_(xStdDev),
                                                                                                                                                                  yStdDev_(yStdDev),
                                                                                                                                                                  multiplicator_(multiplicator){
}

LocalObstacleMovementModel::~LocalObstacleMovementModel() {
}

void LocalObstacleMovementModel::drift(ObstacleStateW& /*state*/, double /*dt*/) const {
}

void LocalObstacleMovementModel::diffuse(ObstacleStateW& state, double /*dt*/) const {
    state.setXPos(state.getXPos() + random_number_generator_.getGaussian(xStdDev_) * multiplicator_);
    state.setYPos(state.getYPos() + random_number_generator_.getGaussian(yStdDev_) * multiplicator_);
}
