#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_

#include <thread>

#include <hardware_interface/robot_hw.h>

#include <bitbots_ros_control/utils.h>
#include <bitbots_ros_control/core_hardware_interface.h>
#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/leds_hardware_interface.h>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/button_hardware_interface.h>
#include <bitbots_ros_control/leds_hardware_interface.h>

namespace bitbots_ros_control {

class WolfgangHardwareInterface : public hardware_interface::RobotHW {
 public:
  explicit WolfgangHardwareInterface(ros::NodeHandle &nh);
  ~WolfgangHardwareInterface();

  bool init(ros::NodeHandle &nh);

  void read(const ros::Time &t, const ros::Duration &dt);

  void write(const ros::Time &t, const ros::Duration &dt);

 private:
  bool create_interfaces(ros::NodeHandle &nh, std::vector<std::pair<std::string, int>> dxl_devices);

  // two dimensional list of all hardware interfaces, sorted by port
  std::vector<std::vector<hardware_interface::RobotHW *>> interfaces_;
  DynamixelServoHardwareInterface servo_interface_;
  ros::Publisher speak_pub_;

  // prevent unnecessary error when power is turned on
  bool first_ping_error_;

  bool only_imu_;
  bool only_pressure_;
};
}

#endif //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
