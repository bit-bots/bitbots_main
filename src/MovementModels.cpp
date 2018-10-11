#include <bitbots_world_model/MovementModels.h>

LocalObstacleMovementModel::LocalObstacleMovementModel() : libPF::MovementModel<ObstacleStateW>() {
}

LocalObstacleMovementModel::~LocalObstacleMovementModel() {
}

void LocalObstacleMovementModel::drift(ObstacleStateW& /*state*/, double /*dt*/) const {
}

void LocalObstacleMovementModel::diffuse(ObstacleStateW& /*state*/, double /*dt*/) const {
}
