#include "bitbots_world_model/ObservationModels.h"

LocalObstacleObservationModel::LocalObstacleObservationModel () : libPF::ObservationModel<ObstacleStateW>() {
}

LocalObstacleObservationModel::~LocalObstacleObservationModel () {
}

double LocalObstacleObservationModel::measure(const ObstacleStateW& state) const {
    return .5;
}


