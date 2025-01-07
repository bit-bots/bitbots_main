//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
#define BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H

#include <particle_filter/ParticleFilter.h>
#include <tf2_ros/buffer.h>

#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/map.hpp>
#include <bitbots_localization/tools.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>

#include "localization_parameters.hpp"

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
                            const bitbots_localization::Params &config, const FieldDimensions &field_dimensions,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   *
   * @param state Reference to the state that has to be weighted.
   * @return weight for the given state.
   */
  double measure(const RobotState &state) const override;

  void set_measurement_lines_pc(sm::msg::PointCloud2 measurement);

  void set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement);

  std::vector<std::pair<double, double>> get_measurement_lines() const;

  std::vector<std::pair<double, double>> get_measurement_goals() const;

  double get_min_weight() const override;

  void clear_measurement();

  bool measurements_available() override;

 private:
  double calculate_weight_for_class(const RobotState &state,
                                    const std::vector<std::pair<double, double>> &last_measurement,
                                    std::shared_ptr<Map> map, double element_weight) const;

  // Measurements
  std::vector<std::pair<double, double>> last_measurement_lines_;
  rclcpp::Time last_stamp_lines;
  std::vector<std::pair<double, double>> last_measurement_goal_;
  rclcpp::Time last_stamp_goals;

  // Reference to the maps for the different classes
  std::shared_ptr<Map> map_lines_;
  std::shared_ptr<Map> map_goals_;

  // Parameters
  bitbots_localization::Params config_;
  FieldDimensions field_dimensions_;

  // TF Buffer, we need a reference to this to estimate how far the robot has moved since the last measurement
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_OBSERVATIONMODEL_H
