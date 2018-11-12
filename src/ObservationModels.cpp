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

LocalFcnnObservationModel::LocalFcnnObservationModel () : libPF::ObservationModel<PositionState>() {
}

LocalFcnnObservationModel::~LocalFcnnObservationModel () {
}

double LocalFcnnObservationModel::measure(const PositionState& state) const {
    if (last_measurement_.empty()) {
        // ROS_ERROR_STREAM("measure function called with empty measurement list. Prevent this by not calling the function of the particle filter on empty measurements.");
        return 1.0;
    }
    std::vector<WeightedMeasurement> weighted_measurements;
    for (bitbots_image_transformer::PixelRelative measurement : last_measurement_) {
        weighted_measurements.push_back(WeightedMeasurement{state.calcDistance(measurement), measurement.value});
    }
    int k = std::min(k_, static_cast<int>(weighted_measurements.size())); // put k in a appropriate bounds
    std::nth_element(weighted_measurements.begin(), weighted_measurements.begin() + (k - 1), weighted_measurements.end(), [](WeightedMeasurement &a, WeightedMeasurement &b){return (a.distance < b.distance);});
    double weighted_weight = 0;
    for (std::vector<WeightedMeasurement>::iterator it = weighted_measurements.begin(); it != weighted_measurements.begin() + k; ++it) {
        weighted_weight += it->weight/it->distance;
    }

    return std::max(min_weight_, weighted_weight);
}

void LocalFcnnObservationModel::set_measurement(bitbots_image_transformer::PixelsRelative measurement) {
    last_measurement_ = measurement.pixels;
}

void LocalFcnnObservationModel::set_min_weight(double min_weight) {
    min_weight_ = min_weight;
}

double LocalFcnnObservationModel::get_min_weight() const {
    return min_weight_;
}

void LocalFcnnObservationModel::set_k(int k) {
    k_ = k;
}

void LocalFcnnObservationModel::clear_measurement() {
    last_measurement_.clear();
}

bool LocalFcnnObservationModel::measurements_available() {
    return (!last_measurement_.empty());
}


