#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BUTTON_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <humanoid_league_msgs/Speak.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <bitbots_buttons/Buttons.h>

#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace bitbots_ros_control
{

class ButtonHardwareInterface : public hardware_interface::RobotHW
{
public:
  ButtonHardwareInterface();
  explicit ButtonHardwareInterface(std::shared_ptr<DynamixelDriver>& driver);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();

private:
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  ros::Publisher button_pub_;
};
}

#endif
