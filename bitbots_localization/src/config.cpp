//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/config.hpp"

namespace bitbots_localization {

Config::Config(std::vector<rclcpp::Parameter> parameters) { Config::update_params(parameters); }

constexpr unsigned int hash(const char* str, int h = 0) { return !str[h] ? 5381 : (hash(str, h + 1) * 33) ^ str[h]; }

void Config::update_params(std::vector<rclcpp::Parameter> parameters) {
  rclcpp::Logger LOGGER = rclcpp::get_logger("localization_param_loading");
  for (rclcpp::Parameter param : parameters) {
    RCLCPP_DEBUG(LOGGER, "Currently loading param: %s", param.get_name().c_str());
    switch (hash(param.get_name().c_str())) {
      case hash("init_mode"):
        this->init_mode = param.as_int();
        break;
      case hash("line_pointcloud_topic"):
        this->line_pointcloud_topic = param.as_string();
        break;
      case hash("goal_topic"):
        this->goal_topic = param.as_string();
        break;
      case hash("fieldboundary_topic"):
        this->fieldboundary_topic = param.as_string();
        break;
      case hash("particle_publishing_topic"):
        this->particle_publishing_topic = param.as_string();
        break;
      case hash("publishing_frequency"):
        this->publishing_frequency = param.as_int();
        break;
      case hash("debug_visualization"):
        this->debug_visualization = param.as_bool();
        break;
      case hash("particle_number"):
        this->particle_number = param.as_int();
        break;
      case hash("resampling_interval"):
        this->resampling_interval = param.as_int();
        break;
      case hash("diffusion_x_std_dev"):
        this->diffusion_x_std_dev = param.as_double();
        break;
      case hash("diffusion_y_std_dev"):
        this->diffusion_y_std_dev = param.as_double();
        break;
      case hash("diffusion_t_std_dev"):
        this->diffusion_t_std_dev = param.as_double();
        break;
      case hash("starting_diffusion"):
        this->starting_diffusion = param.as_double();
        break;
      case hash("starting_steps_with_higher_diffusion"):
        this->starting_steps_with_higher_diffusion = param.as_int();
        break;
      case hash("drift_distance_to_direction"):
        this->drift_distance_to_direction = param.as_double();
        break;
      case hash("drift_rotation_to_direction"):
        this->drift_rotation_to_direction = param.as_double();
        break;
      case hash("drift_distance_to_distance"):
        this->drift_distance_to_distance = param.as_double();
        break;
      case hash("drift_rotation_to_distance"):
        this->drift_rotation_to_distance = param.as_double();
        break;
      case hash("drift_distance_to_rotation"):
        this->drift_distance_to_rotation = param.as_double();
        break;
      case hash("drift_rotation_to_rotation"):
        this->drift_rotation_to_rotation = param.as_double();
        break;
      case hash("max_rotation"):
        this->max_rotation = param.as_double();
        break;
      case hash("max_translation"):
        this->max_translation = param.as_double();
        break;
      case hash("min_weight"):
        this->min_weight = param.as_double();
        break;
      case hash("min_resampling_weight"):
        this->min_resampling_weight = param.as_double();
        break;
      case hash("out_of_field_weight_decrease"):
        this->out_of_field_weight_decrease = param.as_double();
        break;
      case hash("out_of_field_range"):
        this->out_of_field_range = param.as_double();
        break;
      case hash("percentage_best_particles"):
        this->percentage_best_particles = param.as_int();
        break;
      case hash("distance_factor"):
        this->distance_factor = param.as_double();
        break;
      case hash("lines_factor"):
        this->lines_factor = param.as_double();
        break;
      case hash("goals_factor"):
        this->goals_factor = param.as_double();
        break;
      case hash("field_boundary_factor"):
        this->field_boundary_factor = param.as_double();
        break;
      case hash("line_element_confidence"):
        this->line_element_confidence = param.as_double();
        break;
      case hash("goal_element_confidence"):
        this->goal_element_confidence = param.as_double();
        break;
      case hash("field_boundary_element_confidence"):
        this->field_boundary_element_confidence = param.as_double();
        break;
      case hash("min_motion_linear"):
        this->min_motion_linear = param.as_double();
        break;
      case hash("min_motion_angular"):
        this->min_motion_angular = param.as_double();
        break;
      case hash("filter_only_with_motion"):
        this->filter_only_with_motion = param.as_bool();
        break;
      case hash("measurement_out_of_map_punishment"):
        this->measurement_out_of_map_punishment = param.as_double();
        break;
      case hash("field_x"):
        this->field_x = param.as_double();
        break;
      case hash("field_y"):  // field params from blckboard TODO
        this->field_y = param.as_double();
        break;
      case hash("field_padding"):
        this->field_padding = param.as_double();
        break;
      case hash("initial_robot_x1"):
        this->initial_robot_x1 = param.as_double();
        break;
      case hash("initial_robot_y1"):
        this->initial_robot_y1 = param.as_double();
        break;
      case hash("initial_robot_t1"):
        this->initial_robot_t1 = param.as_double();
        break;
      case hash("initial_robot_x2"):
        this->initial_robot_x2 = param.as_double();
        break;
      case hash("initial_robot_y2"):
        this->initial_robot_y2 = param.as_double();
        break;
      case hash("initial_robot_t2"):
        this->initial_robot_t2 = param.as_double();
        break;
      case hash("initial_robot_x"):
        this->initial_robot_x = param.as_double();
        break;
      case hash("initial_robot_y"):
        this->initial_robot_y = param.as_double();
        break;
      case hash("initial_robot_t"):
        this->initial_robot_t = param.as_double();
        break;
    }
  }
}
}  // namespace bitbots_localization
