#include <rclcpp/rclcpp.hpp>
#include "odometry_parameters.hpp"
#include <biped_interfaces/msg/phase.hpp>

#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
using std::placeholders::_1;

namespace bitbots_odometry {

class WalkSupportStateDetector: public rclcpp::Node {
 public:
  WalkSupportStateDetector();
  void loop();
 private:
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_left_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_right_sub_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_foot_pressure_support_state_;

  void pressure_left_callback(bitbots_msgs::msg::FootPressure msg);
  void pressure_right_callback(bitbots_msgs::msg::FootPressure msg);
  int findInflectionPoints(const std::vector<float_t>& function);
  int findLocalMinima(const std::vector<float_t>& function);
  float_t pressure_filtered_left_;
  float_t pressure_filtered_right_;

  biped_interfaces::msg::Phase curr_stance_;

  // Declare parameter listener and struct from the generate_parameter_library
  motion_odometry::ParamListener param_listener_;
  // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  motion_odometry::Params config_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_summed_pressure_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_summed_pressure_right_;

  std::vector<std::pair<float_t, rclcpp::Time>> pressure_left_values_stamped_;
  std::vector<std::pair<float_t, rclcpp::Time>> pressure_right_values_stamped_;

  rclcpp::Time right_ts_down_;
  rclcpp::Time left_ts_down_;


};

}
