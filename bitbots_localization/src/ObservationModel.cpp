//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/ObservationModel.hpp>

namespace bitbots_localization {

RobotPoseObservationModel::RobotPoseObservationModel(std::shared_ptr<Map> map_lines, std::shared_ptr<Map> map_goals,
                                                     std::shared_ptr<Map> map_field_boundary,
                                                     std::shared_ptr<bl::Config> config)
    : particle_filter::ObservationModel<RobotState>() {
  map_lines_ = map_lines;
  map_goals_ = map_goals;
  map_field_boundary_ = map_field_boundary;
  config_ = config;
  particle_filter::ObservationModel<RobotState>::accumulate_weights_ = true;
}

double RobotPoseObservationModel::calculate_weight_for_class(
    const RobotState &state, const std::vector<std::pair<double, double>> &last_measurement, std::shared_ptr<Map> map,
    double element_weight) const {
  double particle_weight_for_class;
  if (!last_measurement.empty()) {
    std::vector<double> ratings = map->Map::provideRating(state, last_measurement);
    particle_weight_for_class = std::accumulate(
        ratings.begin(), ratings.end(), 1.0,
        [element_weight](double a, double b) { return a * ((1 - element_weight) + element_weight * (b / 100)); });
  } else {
    particle_weight_for_class = 0;
  }
  return particle_weight_for_class;
}

double RobotPoseObservationModel::measure(const RobotState &state) const {
  double particle_weight_lines =
      calculate_weight_for_class(state, last_measurement_lines_, map_lines_, config_->line_element_confidence);
  double particle_weight_goal =
      calculate_weight_for_class(state, last_measurement_goal_, map_goals_, config_->goal_element_confidence);
  double particle_weight_field_boundary = calculate_weight_for_class(
      state, last_measurement_field_boundary_, map_field_boundary_, config_->field_boundary_element_confidence);

  double weight =
      (((1 - config_->lines_factor) + config_->lines_factor * particle_weight_lines) *
       ((1 - config_->goals_factor) + config_->goals_factor * particle_weight_goal) *
       ((1 - config_->field_boundary_factor) + config_->field_boundary_factor * particle_weight_field_boundary));

  if (weight < min_weight_) {
    weight = min_weight_;
  }

  // reduce weight if particle is too far outside of the field:
  float range = config_->out_of_field_range;
  if (state.getXPos() > (config_->field_x + config_->field_padding) / 2 + range ||
      state.getXPos() < -(config_->field_x + config_->field_padding) / 2 - range ||
      state.getYPos() > (config_->field_y + config_->field_padding) / 2 + range ||
      state.getYPos() < -(config_->field_y + config_->field_padding) / 2 - range) {
    weight = weight - config_->out_of_field_weight_decrease;
  }

  return weight;  // exponential?
}

void RobotPoseObservationModel::set_measurement_lines_pc(sm::msg::PointCloud2 measurement) {
  for (sm::PointCloud2ConstIterator<float> iter_xyz(measurement, "x"); iter_xyz != iter_xyz.end(); ++iter_xyz) {
    std::pair<double, double> linePolar = cartesianToPolar(iter_xyz[0], iter_xyz[1]);
    last_measurement_lines_.push_back(linePolar);
  }
}

void RobotPoseObservationModel::set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement) {
  // convert to polar
  for (sv3dm::msg::Goalpost &post : measurement.posts) {
    std::pair<double, double> postPolar = cartesianToPolar(post.bb.center.position.x, post.bb.center.position.y);
    last_measurement_goal_.push_back(postPolar);
  }
}

void RobotPoseObservationModel::set_measurement_field_boundary(sv3dm::msg::FieldBoundary measurement) {
  // convert to polar
  for (gm::msg::Point &point : measurement.points) {
    std::pair<double, double> fieldBoundaryPointPolar = cartesianToPolar(point.x, point.y);
    last_measurement_field_boundary_.push_back(fieldBoundaryPointPolar);
  }
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_lines() const {
  return last_measurement_lines_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_goals() const {
  return last_measurement_goal_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_field_boundary() const {
  return last_measurement_field_boundary_;
}

void RobotPoseObservationModel::set_min_weight(double min_weight) { min_weight_ = min_weight; }

double RobotPoseObservationModel::get_min_weight() const { return min_weight_; }

void RobotPoseObservationModel::clear_measurement() {
  last_measurement_lines_.clear();
  last_measurement_goal_.clear();
  last_measurement_field_boundary_.clear();
}

bool RobotPoseObservationModel::measurements_available() {
  bool available = false;
  available |= !last_measurement_lines_.empty();
  available |= !last_measurement_goal_.empty();
  available |= !last_measurement_field_boundary_.empty();
  return available;
}
}  // namespace bitbots_localization
