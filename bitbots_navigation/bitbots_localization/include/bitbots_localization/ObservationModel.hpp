//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
#define BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H

#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/map.hpp>
#include <bitbots_localization/tools.hpp>
#include <localization_parameters.hpp>
#include <particle_filter/ParticleFilter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <soccer_vision_3d_msgs/msg/field_boundary.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>
#include <soccer_vision_3d_msgs/msg/marking_array.hpp>
#include <soccer_vision_3d_msgs/msg/marking_intersection.hpp>

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
                            std::shared_ptr<Map> map_field_boundary, const bitbots_localization::Params &config,
                            const FieldDimensions &field_dimensions);

  /**
   *
   * @param state Reference to the state that has to be weighted.
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

  double get_min_weight() const override;

  void clear_measurement();

  bool measurements_available() override;

 private:
  double calculate_weight_for_class(const RobotState &state,
                                    const std::vector<std::pair<double, double>> &last_measurement,
                                    std::shared_ptr<Map> map, double element_weight) const;

  // Measurements
  std::vector<std::pair<double, double>> last_measurement_lines_;
  std::vector<std::pair<double, double>> last_measurement_goal_;
  std::vector<std::pair<double, double>> last_measurement_field_boundary_;

  // Reference to the maps for the different classes
  std::shared_ptr<Map> map_lines_;
  std::shared_ptr<Map> map_goals_;
  std::shared_ptr<Map> map_field_boundary_;

  // Parameters
  bitbots_localization::Params config_;
  FieldDimensions field_dimensions_;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
