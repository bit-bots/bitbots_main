//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
#define BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H

#include <tf2_ros/buffer.h>

#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/map.hpp>
#include <bitbots_localization/tools.hpp>
#include <localization_parameters.hpp>
#include <particle_filter/ParticleFilter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>

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
                            const bitbots_localization::Params &config, const FieldDimensions &field_dimensions);

  /**
   *
   * @param state Reference to the state that has to be weighted.
   * @return weight for the given state.
   */
  double measure(const RobotState &state) const override;

  void set_measurement_lines_pc(sm::msg::PointCloud2 measurement);

  void set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement);

  const std::vector<std::pair<double, double>> get_measurement_lines() const;

  const std::vector<std::pair<double, double>> get_measurement_goals() const;

  double get_min_weight() const override;

  void clear_measurement();

  bool measurements_available() override;

  void set_movement_since_line_measurement(const tf2::Transform movement);
  void set_movement_since_goal_measurement(const tf2::Transform movement);

 private:
  double calculate_weight_for_class(const RobotState &state,
                                    const std::vector<std::pair<double, double>> &last_measurement,
                                    std::shared_ptr<Map> map, double element_weight,
                                    const tf2::Transform &movement_since_measurement) const;

  // Measurements
  std::vector<std::pair<double, double>> last_measurement_lines_;
  std::vector<std::pair<double, double>> last_measurement_goal_;

  // Movement since last measurement
  tf2::Transform movement_since_line_measurement_ = tf2::Transform::getIdentity();
  tf2::Transform movement_since_goal_measurement_ = tf2::Transform::getIdentity();

  // Reference to the maps for the different classes
  std::shared_ptr<Map> map_lines_;
  std::shared_ptr<Map> map_goals_;

  // Parameters
  bitbots_localization::Params config_;
  FieldDimensions field_dimensions_;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
