#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <humanoid_league_msgs/Audio.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <bitbots_buttons/Buttons.h>

#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

#include <dynamixel_workbench/dynamixel_driver.h>

#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

class ButtonHardwareInterface : public hardware_interface::RobotHW {
 public:
  explicit ButtonHardwareInterface(std::shared_ptr<DynamixelDriver> &driver, int id, std::string topic, int read_rate_);

  bool init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh);
  void read(const ros::Time &t, const ros::Duration &dt);
  void write(const ros::Time &t, const ros::Duration &dt);

 private:
  int counter_;
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  int id_;
  std::string topic_;
  ros::Publisher button_pub_;
  int read_rate_;
  ros::Publisher diagnostic_pub_;
  uint8_t *data_;

};
}

#endif
