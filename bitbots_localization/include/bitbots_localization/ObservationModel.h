//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
#define BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H

#include <particle_filter/ParticleFilter.h>
#include <bitbots_localization/RobotState.h>
#include <bitbots_localization/map.h>
#include <bitbots_localization/config.h>
#include <bitbots_localization/tools.h>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>
#include <soccer_vision_3d_msgs/msg/field_boundary.hpp>
#include <soccer_vision_3d_msgs/msg/marking_array.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost.hpp>
#include <soccer_vision_3d_msgs/msg/marking_intersection.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace sm = sensor_msgs;
namespace bl = bitbots_localization;
namespace sv3dm = soccer_vision_3d_msgs;

namespace bitbots_localization {
class RobotPoseObservationModel : public particle_filter::ObservationModel<RobotState> {

 public:

  /**
   * empty
   */
  RobotPoseObservationModel(std::shared_ptr<Map> map_lines, std::shared_ptr<Map> map_goals,
                            std::shared_ptr<Map> map_field_boundary, std::shared_ptr<Map> map_corners,
                            std::shared_ptr<Map> map_t_crossings, std::shared_ptr<Map> map_crosses,
                            std::shared_ptr<bl::Config> config);

  /**
   *
   * @param state Reference to the state that has to be weightened.
   * @return weight for the given state.
   */
  double measure(const RobotState &state) const override;

  void set_measurement_lines_pc(sm::msg::PointCloud2 measurement);

  void set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement);

  void set_measurement_field_boundary(sv3dm::msg::FieldBoundary measurement);

  void set_measurement_markings(sv3dm::msg::MarkingArray measurement);

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

  static int number_of_effective_measurements_;

  void set_min_weight(double min_weight);

  double get_min_weight() const override;

  void clear_measurement();

  bool measurements_available() override;

 private:

  double calculate_weight_for_class(
    const RobotState &state,
    const std::vector<std::pair<double, double>> &last_measurement,
    std::shared_ptr<Map> map,
    double element_weight) const;

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

  std::shared_ptr<bl::Config> config_;

};

double RobotPoseObservationModel::number_lines = 0;
double RobotPoseObservationModel::number_goals = 0;
double RobotPoseObservationModel::number_fb_points = 0;
double RobotPoseObservationModel::number_corners = 0;
double RobotPoseObservationModel::number_tcrossings = 0;
double RobotPoseObservationModel::number_crosses = 0;
int RobotPoseObservationModel::number_of_effective_measurements_ = 0;

};

#endif //BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
