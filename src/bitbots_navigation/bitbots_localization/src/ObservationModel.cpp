//
// Created by judith on 09.03.19.
//

#include <bitbots_localization/ObservationModel.hpp>

namespace bitbots_localization {

RobotPoseObservationModel::RobotPoseObservationModel(std::shared_ptr<Map> map_lines, std::shared_ptr<Map> map_goals,
                                                     const bitbots_localization::Params& config,
                                                     const FieldDimensions& field_dimensions)
    : particle_filter::ObservationModel<RobotState>(),
      map_lines_(map_lines),
      map_goals_(map_goals),
      config_(config),
      field_dimensions_(field_dimensions) {
  particle_filter::ObservationModel<RobotState>::accumulate_weights_ = true;
}

double RobotPoseObservationModel::calculate_weight_for_class(
    const RobotState& state, const std::vector<std::pair<double, double>>& last_measurement, std::shared_ptr<Map> map,
    double element_weight, const tf2::Transform& movement_since_measurement) const {
  double particle_weight_for_class;
  if (!last_measurement.empty()) {
    // Subtract (reverse) the movement from our state to get the hypothetical state at the time of the measurement
    const RobotState state_at_measurement(state.getTransform() * movement_since_measurement.inverse());
    // Get the ratings for for all points in the measurement based on the map
    const std::vector<double> ratings = map->Map::provideRating(state_at_measurement, last_measurement);
    // Take the average of the ratings (functional)
    particle_weight_for_class = std::pow(std::accumulate(ratings.begin(), ratings.end(), 0.0) / ratings.size(), 2);
  } else {
    particle_weight_for_class = 0;
  }
  return particle_weight_for_class;
}

double RobotPoseObservationModel::measure(const RobotState& state) const {
  double particle_weight_lines =
      calculate_weight_for_class(state, last_measurement_lines_, map_lines_,
                                 config_.particle_filter.confidences.line_element, movement_since_line_measurement_);
  double particle_weight_goal =
      calculate_weight_for_class(state, last_measurement_goal_, map_goals_,
                                 config_.particle_filter.confidences.goal_element, movement_since_goal_measurement_);

  // Get relevant config values
  auto scoring_config = config_.particle_filter.scoring;

  // Calculate weight for the particle
  double weight = (((1 - scoring_config.lines.factor) + scoring_config.lines.factor * particle_weight_lines) *
                   ((1 - scoring_config.goal.factor) + scoring_config.goal.factor * particle_weight_goal));

  if (weight < config_.particle_filter.weighting.min_weight) {
    weight = config_.particle_filter.weighting.min_weight;
  }

  // reduce weight if particle is too far outside of the field:
  float range = config_.particle_filter.weighting.out_of_field_range;
  if (state.getXPos() > (field_dimensions_.x + field_dimensions_.padding) / 2 + range ||
      state.getXPos() < -(field_dimensions_.x + field_dimensions_.padding) / 2 - range ||
      state.getYPos() > (field_dimensions_.y + field_dimensions_.padding) / 2 + range ||
      state.getYPos() < -(field_dimensions_.y + field_dimensions_.padding) / 2 - range) {
    weight = weight - config_.particle_filter.weighting.out_of_field_weight_decrease;
  }

  return weight;  // exponential?
}

void RobotPoseObservationModel::set_measurement_lines_pc(sm::msg::PointCloud2 measurement) {
  // convert to polar
  for (sm::PointCloud2ConstIterator<float> iter_xyz(measurement, "x"); iter_xyz != iter_xyz.end(); ++iter_xyz) {
    std::pair<double, double> linePolar = cartesianToPolar(iter_xyz[0], iter_xyz[1]);
    last_measurement_lines_.push_back(linePolar);
  }
}

void RobotPoseObservationModel::set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement) {
  // convert to polar
  for (sv3dm::msg::Goalpost& post : measurement.posts) {
    std::pair<double, double> postPolar = cartesianToPolar(post.bb.center.position.x, post.bb.center.position.y);
    last_measurement_goal_.push_back(postPolar);
  }
}

const std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_lines() const {
  return last_measurement_lines_;
}

const std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_goals() const {
  return last_measurement_goal_;
}

double RobotPoseObservationModel::get_min_weight() const { return config_.particle_filter.weighting.min_weight; }

void RobotPoseObservationModel::clear_measurement() {
  last_measurement_lines_.clear();
  last_measurement_goal_.clear();
}

bool RobotPoseObservationModel::measurements_available() {
  bool available = false;
  available |= !last_measurement_lines_.empty();
  available |= !last_measurement_goal_.empty();
  return available;
}

void RobotPoseObservationModel::set_movement_since_line_measurement(const tf2::Transform movement) {
  movement_since_line_measurement_ = movement;
}

void RobotPoseObservationModel::set_movement_since_goal_measurement(const tf2::Transform movement) {
  movement_since_goal_measurement_ = movement;
}
}  // namespace bitbots_localization
