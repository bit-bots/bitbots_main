#include "bitbots_world_model/ObservationModels.h"

LocalObstacleObservationModel::LocalObstacleObservationModel () : libPF::ObservationModel<PositionStateW>() {
}

LocalObstacleObservationModel::~LocalObstacleObservationModel () {
}

double LocalObstacleObservationModel::measure(const PositionStateW& state) const {
    if (last_measurement_.empty()) {
        ROS_ERROR_STREAM("measure function called with empty measurement list. Prevent this by not calling the function of the particle filter on empty measurements.");
        return 1.0;
    }
    return std::max(min_weight_, 1/state.calcDistance(*std::min_element(last_measurement_.begin(), last_measurement_.end(), [&state](PositionStateW a, PositionStateW b) {return state.calcDistance(a) < state.calcDistance(b); })));
}

void LocalObstacleObservationModel::set_measurement(std::vector<PositionStateW> measurement) {
    last_measurement_ = measurement;
}

void LocalObstacleObservationModel::set_min_weight(double min_weight) {
    min_weight_ = min_weight;
}

double LocalObstacleObservationModel::get_min_weight() const {
    return min_weight_;
}

void LocalObstacleObservationModel::clear_measurement() {
    last_measurement_.clear();
}

bool LocalObstacleObservationModel::measurements_available() {
    return (!last_measurement_.empty());
}

LocalRobotObservationModel::LocalRobotObservationModel () : libPF::ObservationModel<PositionState>() {
}

LocalRobotObservationModel::~LocalRobotObservationModel () {
}

double LocalRobotObservationModel::measure(const PositionState& state) const {
    if (last_measurement_.empty()) {
        ROS_ERROR_STREAM("measure function called with empty measurement list. Prevent this by not calling the function of the particle filter on empty measurements.");
        return 1.0;
    }
    return std::max(min_weight_, 1/state.calcDistance(*std::min_element(last_measurement_.begin(), last_measurement_.end(), [&state](PositionState a, PositionState b) {return state.calcDistance(a) < state.calcDistance(b); })));
}

void LocalRobotObservationModel::set_measurement(std::vector<PositionState> measurement) {
    last_measurement_ = measurement;
}

void LocalRobotObservationModel::set_min_weight(double min_weight) {
    min_weight_ = min_weight;
}

double LocalRobotObservationModel::get_min_weight() const {
    return min_weight_;
}

void LocalRobotObservationModel::clear_measurement() {
    last_measurement_.clear();
}

bool LocalRobotObservationModel::measurements_available() {
    return (!last_measurement_.empty());
}

