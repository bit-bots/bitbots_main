#include "bitbots_world_model/ObservationModels.h"

LocalObstacleObservationModel::LocalObstacleObservationModel () : libPF::ObservationModel<ObstacleStateW>() {
}

LocalObstacleObservationModel::~LocalObstacleObservationModel () {
}

double LocalObstacleObservationModel::measure(const ObstacleStateW& state) const {
    return .5;
}

void LocalObstacleObservationModel::set_measurement(std::vector<ObstacleStateW> measurement) {
    last_measurement_ = measurement;
}
