#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace bitbots_ros_control {

class BitFootHardwareInterface : public bitbots_ros_control::HardwareInterface {
 public:
  explicit BitFootHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver>& driver, int id,
                                    std::string topic_name, std::string name);

  bool init();

  void read(const rclcpp::Time& t, const rclcpp::Duration& dt);

  void write(const rclcpp::Time& t, const rclcpp::Duration& dt);

 private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<DynamixelDriver> driver_;

  // always keep the lasts values to check if they different
  std::vector<std::vector<double>> current_pressure_;

  rclcpp::Publisher<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_pub_;

  int id_;
  std::string topic_name_;
  std::string name_;
  bitbots_msgs::msg::FootPressure msg_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  std::array<uint8_t, 16> data_;
};
}  // namespace bitbots_ros_control
#endif
