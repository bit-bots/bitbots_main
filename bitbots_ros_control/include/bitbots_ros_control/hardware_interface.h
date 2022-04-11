//
// Created by 10bestman on 08.04.22.
//

#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_HARDWARE_INTERFACE_H_
namespace bitbots_ros_control {

class HardwareInterface {
 public:
  virtual bool init() = 0;

  virtual void read(const rclcpp::Time &t, const rclcpp::Duration &dt) {};

  virtual void write(const rclcpp::Time &t, const rclcpp::Duration &dt) {};

  virtual ~HardwareInterface() {};
};
}
#endif //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_HARDWARE_INTERFACE_H_
