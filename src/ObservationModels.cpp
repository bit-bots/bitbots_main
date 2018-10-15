#include "bitbots_world_model/ObservationModels.h"

LocalObstacleObservationModel::LocalObstacleObservationModel () : libPF::ObservationModel<ObstacleStateW>() {
}

LocalObstacleObservationModel::~LocalObstacleObservationModel () {
}

double LocalObstacleObservationModel::measure(const ObstacleStateW& state) const {
    if (last_measurement_.empty()) {
        ROS_ERROR_STREAM("measure function called with empty measurement list. Prevent this by not calling the function of the particle filter on empty measurements.");
        return 1.0;
    }
    return state.calcDistance(*std::min_element(last_measurement_.begin(), last_measurement_.end(), [&state](ObstacleStateW a, ObstacleStateW b) {return state.calcDistance(a) < state.calcDistance(b); }));
}

void LocalObstacleObservationModel::set_measurement(std::vector<ObstacleStateW> measurement) {
    last_measurement_ = measurement;
}

void LocalObstacleObservationModel::clear_measurement() {
    last_measurement_.clear();
}

bool LocalObstacleObservationModel::measurements_available() {
    return (!last_measurement_.empty());
}

LocalRobotObservationModel::LocalRobotObservationModel () : libPF::ObservationModel<ObstacleState>() {
}

LocalRobotObservationModel::~LocalRobotObservationModel () {
}

double LocalRobotObservationModel::measure(const ObstacleState& state) const {
    if (last_measurement_.empty()) {
        ROS_ERROR_STREAM("measure function called with empty measurement list. Prevent this by not calling the function of the particle filter on empty measurements.");
        return 1.0;
    }
    return state.calcDistance(*std::min_element(last_measurement_.begin(), last_measurement_.end(), [&state](ObstacleState a, ObstacleState b) {return state.calcDistance(a) < state.calcDistance(b); }));
}

void LocalRobotObservationModel::set_measurement(std::vector<ObstacleState> measurement) {
    last_measurement_ = measurement;
}

void LocalRobotObservationModel::clear_measurement() {
    last_measurement_.clear();
}

bool LocalRobotObservationModel::measurements_available() {
    return (!last_measurement_.empty());
}

