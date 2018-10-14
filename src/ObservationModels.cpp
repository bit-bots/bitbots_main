#include "bitbots_world_model/ObservationModels.h"

LocalObstacleObservationModel::LocalObstacleObservationModel () : libPF::ObservationModel<ObstacleStateW>() {
}

LocalObstacleObservationModel::~LocalObstacleObservationModel () {
}

double LocalObstacleObservationModel::measure(const ObstacleStateW& state) const {
    return state.calcDistance(*std::min_element(last_measurement_.begin(), last_measurement_.end(), [&state](ObstacleStateW a, ObstacleStateW b) {return state.calcDistance(a) < state.calcDistance(b); }));
}

void LocalObstacleObservationModel::set_measurement(std::vector<ObstacleStateW> measurement) {
    last_measurement_ = measurement;
}
