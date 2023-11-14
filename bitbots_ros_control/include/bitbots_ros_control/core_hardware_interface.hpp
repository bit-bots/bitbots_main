#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_CORE_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_CORE_HARDWARE_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_ros_control/hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

namespace bitbots_ros_control {

class CoreHardwareInterface : public bitbots_ros_control::HardwareInterface {
 public:
  explicit CoreHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver, int id,
                                 int read_rate);

  bool get_power_status();

  bool init();

  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);

  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);

 private:
  rclcpp::Node::SharedPtr nh_;
  bool switch_power(std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

  std::shared_ptr<DynamixelDriver> driver_;

  int id_;
  int read_rate_;
  int read_counter_;
  uint8_t *data_;

  bool requested_power_status_;
  bool last_read_successful_;
  std_msgs::msg::Bool power_switch_status_;
  std_msgs::msg::Bool power_control_status_;
  std_msgs::msg::Float64 VCC_;
  std_msgs::msg::Float64 VBAT_;
  std_msgs::msg::Float64MultiArray VBAT_individual_;
  std_msgs::msg::Float64 VEXT_;
  std_msgs::msg::Float64 VDXL_;
  std_msgs::msg::Float64 current_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr power_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vcc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vbat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vbat_individual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vext_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vdxl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_switch_service_;
};
}  // namespace bitbots_ros_control
#endif
