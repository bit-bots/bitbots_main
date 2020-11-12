#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_BITFOOT_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <humanoid_league_msgs/Audio.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <bitbots_msgs/FootPressure.h>

#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace bitbots_ros_control {

class BitFootHardwareInterface : public hardware_interface::RobotHW {
 public:
  explicit BitFootHardwareInterface(std::shared_ptr<DynamixelDriver> &driver,
                                    int id,
                                    std::string topic_name,
                                    std::string name);

  bool init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) override;

  void read(const ros::Time &t, const ros::Duration &dt) override;

  void write(const ros::Time &t, const ros::Duration &dt) override;

 private:
  ros::NodeHandle nh_;

  std::shared_ptr<DynamixelDriver> driver_;

  // always keep the lasts values to check if they different
  std::vector<std::vector<double>> current_pressure_;

  ros::Publisher pressure_pub_;

  int id_;
  std::string topic_name_;
  std::string name_;
  bitbots_msgs::FootPressure msg_;
  ros::Publisher diagnostic_pub_;
  uint8_t *data_;

};
}
#endif
