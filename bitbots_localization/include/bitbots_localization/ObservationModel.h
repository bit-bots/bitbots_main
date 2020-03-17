//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
#define BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H

#include <particle_filter/ParticleFilter.h>
#include <bitbots_localization/RobotState.h>
#include <bitbots_localization/map.h>
#include <bitbots_localization/tools.h>
#include <humanoid_league_msgs/LineInformationRelative.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <humanoid_league_msgs/PixelsRelative.h>
#include <humanoid_league_msgs/FieldBoundaryRelative.h>

namespace hlm = humanoid_league_msgs;

class RobotPoseObservationModel : public particle_filter::ObservationModel<RobotState> {

 public:

  /**
   * empty
   */
  RobotPoseObservationModel(std::shared_ptr<Map> map_lines, std::shared_ptr<Map> map_goals,
                            std::shared_ptr<Map> map_field_boundary, std::shared_ptr<Map> map_corners,
                            std::shared_ptr<Map> map_t_crossings, std::shared_ptr<Map> map_crosses);

  /**
   *
   * @param state Reference to the state that has to be weightened.
   * @return weight for the given state.
   */
  double measure(const RobotState &state) const override;

  void set_measurement_lines(hlm::LineInformationRelative measurement);

  void set_measurement_goal(hlm::GoalRelative measurement);

  void set_measurement_field_boundary(hlm::FieldBoundaryRelative measurement);

  void set_measurement_corners(hlm::PixelsRelative measurement);

  void set_measurement_t_crossings(hlm::PixelsRelative measurement);

  void set_measurement_crosses(hlm::PixelsRelative measurement);

  std::vector<std::pair<double, double>> get_measurement_lines() const;

  std::vector<std::pair<double, double>> get_measurement_goals() const;

  std::vector<std::pair<double, double>> get_measurement_field_boundary() const;

  std::vector<std::pair<double, double>> get_measurement_corners() const;

  std::vector<std::pair<double, double>> get_measurement_t_crossings() const;

  std::vector<std::pair<double, double>> get_measurement_crosses() const;

  static double number_lines;
  static double number_goals;
  static double number_fb_points;
  static double number_corners;
  static double number_tcrossings;
  static double number_crosses;

  void set_min_weight(double min_weight);

  double get_min_weight() const override;

  void clear_measurement();

  bool measurements_available() override;

 private:

  std::vector<std::pair<double, double>> last_measurement_lines_;

  std::vector<std::pair<double, double>> last_measurement_goal_;

  std::vector<std::pair<double, double>> last_measurement_field_boundary_;

  std::vector<std::pair<double, double>> last_measurement_corners_;

  std::vector<std::pair<double, double>> last_measurement_t_crossings_;

  std::vector<std::pair<double, double>> last_measurement_crosses_;

  double min_weight_ = 0;

  std::shared_ptr<Map> map_lines_;
  std::shared_ptr<Map> map_goals_;
  std::shared_ptr<Map> map_field_boundary_;
  std::shared_ptr<Map> map_corners_;
  std::shared_ptr<Map> map_t_crossings_;
  std::shared_ptr<Map> map_crosses_;

};

double RobotPoseObservationModel::number_lines = 0;
double RobotPoseObservationModel::number_goals = 0;
double RobotPoseObservationModel::number_fb_points = 0;
double RobotPoseObservationModel::number_corners = 0;
double RobotPoseObservationModel::number_tcrossings = 0;
double RobotPoseObservationModel::number_crosses = 0;

#endif //BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
