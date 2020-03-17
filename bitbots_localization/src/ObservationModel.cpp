//
// Created by judith on 09.03.19.
//


#include <bitbots_localization/ObservationModel.h>

RobotPoseObservationModel::RobotPoseObservationModel(std::shared_ptr<Map> map_lines,
                                                     std::shared_ptr<Map> map_goals,
                                                     std::shared_ptr<Map> map_field_boundary,
                                                     std::shared_ptr<Map> map_corners,
                                                     std::shared_ptr<Map> map_t_crossings,
                                                     std::shared_ptr<Map> map_crosses,
                                                     bl::LocalizationConfig &config)
    : particle_filter::ObservationModel<RobotState>() {
  map_lines_ = map_lines;
  map_goals_ = map_goals;
  map_field_boundary_ = map_field_boundary;
  map_corners_ = map_corners;
  map_t_crossings_ = map_t_crossings;
  map_crosses_ = map_crosses;
  config_ = config;
  particle_filter::ObservationModel<RobotState>::accumulate_weights_ = true;

}

double RobotPoseObservationModel::measure(const RobotState &state) const {

  std::vector<double> lineRatings;
  std::vector<double> goalRatings;
  std::vector<double> fieldBoundaryRatings;
  std::vector<double> cornersRating;
  std::vector<double> tCrossingsRating;
  std::vector<double> crossesRating;

  double particle_weight_lines = 0;
  double particle_weight_goal = 0;
  double particle_weight_field_boundary = 0;
  double particle_weight_corners = 0;
  double particle_weight_t_crossings = 0;
  double particle_weight_crosses = 0;

  int number_of_effective_measurements = 0;

  if (!last_measurement_lines_.empty()) {
    number_of_effective_measurements += 1;
    lineRatings = map_lines_->Map::provideRating(state, last_measurement_lines_);
    for (double rating : lineRatings) {
      particle_weight_lines += rating;
    }
    particle_weight_lines = particle_weight_lines /
        lineRatings.size();
  }

  if (!last_measurement_goal_.empty()) {
    number_of_effective_measurements += 1;
    goalRatings = map_goals_->Map::provideRating(state, last_measurement_goal_);
    for (double rating : goalRatings) {
      particle_weight_goal += rating;
    }
    particle_weight_goal = particle_weight_goal / goalRatings.size();

  }

  if (!last_measurement_field_boundary_.empty()) {
    number_of_effective_measurements += 1;
    fieldBoundaryRatings = map_field_boundary_->Map::provideRating(state, last_measurement_field_boundary_);
    for (double rating : fieldBoundaryRatings) {
      particle_weight_field_boundary += rating;
    }
    particle_weight_field_boundary = particle_weight_field_boundary / fieldBoundaryRatings.size();
  }

  if (!last_measurement_corners_.empty()) {
    number_of_effective_measurements += 1;
    cornersRating = map_corners_->Map::provideRating(state, last_measurement_corners_);
    for (double rating : cornersRating) {
      particle_weight_corners += rating;
    }
    particle_weight_corners = particle_weight_corners / cornersRating.size();

  }

  if (!last_measurement_t_crossings_.empty()) {
    number_of_effective_measurements += 1;
    tCrossingsRating = map_t_crossings_->Map::provideRating(state, last_measurement_t_crossings_);
    for (double rating : tCrossingsRating) {
      particle_weight_t_crossings += rating;
    }
    particle_weight_t_crossings = particle_weight_t_crossings / tCrossingsRating.size();

  }

  if (!last_measurement_crosses_.empty()) {
    number_of_effective_measurements += 1;
    crossesRating = map_crosses_->Map::provideRating(state, last_measurement_crosses_);
    for (double rating : crossesRating) {
      particle_weight_crosses += rating;
    }
    particle_weight_crosses = particle_weight_crosses / crossesRating.size();

  }

  number_lines = last_measurement_lines_.size();
  number_goals = last_measurement_goal_.size();
  number_fb_points = last_measurement_field_boundary_.size();
  number_corners = last_measurement_corners_.size();
  number_tcrossings = last_measurement_t_crossings_.size();
  number_crosses = last_measurement_crosses_.size();


  // TODO use config params
  double weight = (number_of_effective_measurements == 0) ? 0 : (
      ( particle_weight_lines * config_.lines_factor + 
        particle_weight_goal * config_.goal_factor + 
        particle_weight_field_boundary * config_.field_boundary_factor +
        particle_weight_corners * config_.corners_factor + 
        particle_weight_t_crossings * config_.tcrossings_factor + 
        particle_weight_crosses * config_.crosses_factor) /
          number_of_effective_measurements); // TODO evaluate this devision
  if (weight < min_weight_) {
    weight = min_weight_;
  }
  return weight; //exponential?
}

void RobotPoseObservationModel::set_measurement_lines(hlm::LineInformationRelative measurement) {
  // convert to polar
  last_measurement_lines_.clear();
  for (hlm::LineSegmentRelative &segment : measurement.segments) {
    std::pair<double, double> linePolar = cartesianToPolar(segment.start.x, segment.start.y);
    last_measurement_lines_.push_back(linePolar);
  }
}

void RobotPoseObservationModel::set_measurement_goal(hlm::GoalRelative measurement) {
  // convert to polar
  last_measurement_goal_.clear();
  if (measurement.left_post.x != 0 && measurement.left_post.y != 0) {
    std::pair<double, double> postOnePolar = cartesianToPolar(measurement.left_post.x, measurement.left_post.y);
    last_measurement_goal_.push_back(postOnePolar);
    if (measurement.left_post.x != measurement.right_post.x && measurement.left_post.y != measurement.right_post.y) {
      std::pair<double, double> postTwoPolar = cartesianToPolar(measurement.right_post.x, measurement.right_post.y);
      last_measurement_goal_.push_back(postTwoPolar);
    }

  }

}

void RobotPoseObservationModel::set_measurement_field_boundary(hlm::FieldBoundaryRelative measurement) {
  // convert to polar
  last_measurement_field_boundary_.clear();
  for (gm::Point &point : measurement.field_boundary_points) {
    std::pair<double, double> fieldBoundaryPointPolar = cartesianToPolar(point.x,
                                                                         point.y); // z is 0
    last_measurement_field_boundary_.push_back(fieldBoundaryPointPolar);
  }
}

void RobotPoseObservationModel::set_measurement_corners(hlm::PixelsRelative measurement) {
  // convert to polar
  last_measurement_corners_.clear();
  for (hlm::PixelRelative &pixel : measurement.pixels) {
    std::pair<double, double> cornerPolar = cartesianToPolar(pixel.position.x, pixel.position.y); // z is 0
    last_measurement_corners_.push_back(cornerPolar);
  }
}

void RobotPoseObservationModel::set_measurement_t_crossings(hlm::PixelsRelative measurement) {
  // convert to polar
  last_measurement_t_crossings_.clear();
  for (hlm::PixelRelative &pixel : measurement.pixels) {
    std::pair<double, double> tcrossingsPolar = cartesianToPolar(pixel.position.x, pixel.position.y); // z is 0
    last_measurement_t_crossings_.push_back(tcrossingsPolar);
  }
}

void RobotPoseObservationModel::set_measurement_crosses(hlm::PixelsRelative measurement) {
  // convert to polar
  last_measurement_crosses_.clear();
  for (hlm::PixelRelative &pixel : measurement.pixels) {
    std::pair<double, double> cornerPolar = cartesianToPolar(pixel.position.x, pixel.position.y); // z is 0
    last_measurement_crosses_.push_back(cornerPolar);
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

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_corners() const {
  return last_measurement_corners_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_t_crossings() const {
  return last_measurement_t_crossings_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_crosses() const {
  return last_measurement_crosses_;
}

void RobotPoseObservationModel::set_min_weight(double min_weight) {
  min_weight_ = min_weight;
}

double RobotPoseObservationModel::get_min_weight() const {
  return min_weight_;
}

void RobotPoseObservationModel::clear_measurement() {
  last_measurement_lines_.clear();
  last_measurement_goal_.clear();
  last_measurement_field_boundary_.clear();
  last_measurement_corners_.clear();
  last_measurement_t_crossings_.clear();
  last_measurement_crosses_.clear();
}

bool RobotPoseObservationModel::measurements_available() {
  return (!last_measurement_lines_.empty());
}

