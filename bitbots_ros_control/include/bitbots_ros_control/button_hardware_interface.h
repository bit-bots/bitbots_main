#ifndef BUTTON_HARWARE_INTERFACE_H
#define BUTTON_HARWARE_INTERFACE_H

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
  ButtonHardwareInterface(boost::shared_ptr<DynamixelDriver>& driver);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();

private:
  ros::NodeHandle _nh;
  boost::shared_ptr<DynamixelDriver> _driver;
  ros::Publisher _button_pub;
};
}

#endif
