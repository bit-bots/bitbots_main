#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_LEDS_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_LEDS_HARDWARE_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_msgs/srv/leds.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace bitbots_ros_control {

class LedsHardwareInterface : public bitbots_ros_control::HardwareInterface {
 public:
  LedsHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver, uint8_t id,
                        uint8_t num_leds, uint8_t start_number);

  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);

 private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  uint8_t id_;
  uint8_t start_number_;

  bool write_leds_ = false;
  std::vector<std_msgs::msg::ColorRGBA> leds_;

  rclcpp::Service<bitbots_msgs::srv::Leds>::SharedPtr leds_service_;
  void setLeds(const std::shared_ptr<bitbots_msgs::srv::Leds::Request> req,
               std::shared_ptr<bitbots_msgs::srv::Leds::Response> resp);
  void ledCb0(std_msgs::msg::ColorRGBA msg);
  void ledCb1(std_msgs::msg::ColorRGBA msg);
  void ledCb2(std_msgs::msg::ColorRGBA msg);

  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr sub0_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr sub2_;
};
}  // namespace bitbots_ros_control
#endif
