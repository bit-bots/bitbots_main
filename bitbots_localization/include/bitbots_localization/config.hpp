//
// Created by judith on 08.03.19.
//

#ifndef BITBOTS_LOCALIZATION_CONFIG_H
#define BITBOTS_LOCALIZATION_CONFIG_H

#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bitbots_localization {
/**
 * @class Config
 * @brief Stores a configuration of the localization
 */
class Config {
 public:
  /**
   * @param parameters of the localization.
   */
  explicit Config(std::vector<rclcpp::Parameter> parameters);

  /**
   * @param parameters of the localization.
   */
  void update_params(std::vector<rclcpp::Parameter> parameters);

  int init_mode = 0;
  std::string line_pointcloud_topic = "";
  std::string goal_topic = "";
  std::string fieldboundary_topic = "";
  std::string particle_publishing_topic = "";
  int publishing_frequency = 10;
  bool debug_visualization = true;
  int particle_number = 1;
  int resampling_interval = 0;
  double diffusion_x_std_dev = 0.0;
  double diffusion_y_std_dev = 0.0;
  double diffusion_t_std_dev = 0.0;
  double diffusion_multiplicator = 0.0;
  double starting_diffusion = 0.0;
  int starting_steps_with_higher_diffusion = 0;
  double drift_distance_to_direction = 0.0;
  double drift_rotation_to_direction = 0.0;
  double drift_distance_to_distance = 0.0;
  double drift_rotation_to_distance = 0.0;
  double drift_distance_to_rotation = 0.0;
  double drift_rotation_to_rotation = 0.0;
  double max_rotation = 0.0;
  double max_translation = 0.0;
  double min_weight = 0.0;
  double min_resampling_weight = 0.0;
  double out_of_field_weight_decrease = 0.0;
  double out_of_field_range = 0.0;
  int percentage_best_particles = 0;
  double distance_factor = 0.0;
  double lines_factor = 0.0;
  double goals_factor = 0.0;
  double field_boundary_factor = 0.0;
  double line_element_confidence = 0.0;
  double goal_element_confidence = 0.0;
  double field_boundary_element_confidence = 0.0;
  double min_motion_linear = 0.0;
  double min_motion_angular = 0.0;
  bool filter_only_with_motion = false;
  double measurement_out_of_map_punishment = 0.0;
  // Field secific settings
  double field_x = 0;
  double field_y = 0;
  double field_padding = 0;
  double initial_robot_x1 = 0.0;
  double initial_robot_y1 = 0.0;
  double initial_robot_t1 = 0.0;
  double initial_robot_x2 = 0.0;
  double initial_robot_y2 = 0.0;
  double initial_robot_t2 = 0.0;
  double initial_robot_x = 0.0;
  double initial_robot_y = 0.0;
  double initial_robot_t = 0.0;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_CONFIG_H
