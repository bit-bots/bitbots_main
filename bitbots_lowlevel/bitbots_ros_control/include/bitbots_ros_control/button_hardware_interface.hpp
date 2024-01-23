#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_msgs/msg/buttons.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace bitbots_ros_control {

class ButtonHardwareInterface : public bitbots_ros_control::HardwareInterface {
 public:
  explicit ButtonHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver, int id,
                                   std::string topic, int read_rate_);

  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);

 private:
  rclcpp::Node::SharedPtr nh_;
  int counter_;

  std::shared_ptr<DynamixelDriver> driver_;
  int id_;
  std::string topic_;
  rclcpp::Publisher<bitbots_msgs::msg::Buttons>::SharedPtr button_pub_;
  int read_rate_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  uint8_t *data_;
};
}  // namespace bitbots_ros_control

#endif
