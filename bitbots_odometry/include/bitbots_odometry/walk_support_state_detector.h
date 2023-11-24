#include <rclcpp/rclcpp.hpp>
#include "odometry_parameters.hpp"
#include <biped_interfaces/msg/phase.hpp>

#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <std_msgs/msg/float64.hpp>
using std::placeholders::_1;

namespace bitbots_odometry {

class WalkSupportStateDetector: public rclcpp::Node {
 public:
  WalkSupportStateDetector();
  void loop();
 private:
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_l_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_r_sub_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_foot_pressure_support_state_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pressure_debug_l_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_foot_pressure_debug_r_;
  void pressure_l_callback(bitbots_msgs::msg::FootPressure msg);
  void pressure_r_callback(bitbots_msgs::msg::FootPressure msg);
  int curr_stand_left_;
  int curr_stand_right_;
  int prev_stand_right_;
  int prev_stand_left_;
  float_t pressure_filtered_left_;
  float_t pressure_filtered_right_;
  float_t k;
  float_t m;
  long step_duration_r_;
  rclcpp::Time up_r_;

  long step_duration_l_;
  rclcpp::Time up_l_;
  biped_interfaces::msg::Phase curr_stance_;

  // Declare parameter listener and struct from the generate_parameter_library
  motion_odometry::ParamListener param_listener_;
  // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  motion_odometry::Params config_;
};

}
