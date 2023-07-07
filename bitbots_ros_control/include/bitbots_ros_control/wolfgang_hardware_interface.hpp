#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_

#include <bitbots_ros_control/bitfoot_hardware_interface.hpp>
#include <bitbots_ros_control/button_hardware_interface.hpp>
#include <bitbots_ros_control/core_hardware_interface.hpp>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <bitbots_ros_control/imu_hardware_interface.hpp>
#include <bitbots_ros_control/leds_hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>
#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace bitbots_ros_control {

class WolfgangHardwareInterface {
 public:
  explicit WolfgangHardwareInterface(rclcpp::Node::SharedPtr nh);

  bool init();

  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);

  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);

 private:
  bool create_interfaces(std::vector<std::pair<std::string, int>> dxl_devices);
  rclcpp::Node::SharedPtr nh_;

  // two dimensional list of all hardware interfaces, sorted by port
  std::vector<std::vector<bitbots_ros_control::HardwareInterface *>> interfaces_;
  DynamixelServoHardwareInterface servo_interface_;
  rclcpp::Publisher<humanoid_league_msgs::msg::Audio>::SharedPtr speak_pub_;

  // prevent unnecessary error when power is turned on
  bool first_ping_error_;

  bool only_imu_;
  bool only_pressure_;
  bool core_present_;
  bool current_power_status_;
  bool last_power_status_;
  CoreHardwareInterface *core_interface_;
};
}  // namespace bitbots_ros_control

#endif  // BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
