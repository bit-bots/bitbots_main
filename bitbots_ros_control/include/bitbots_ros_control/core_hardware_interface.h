#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_CORE_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_CORE_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hardware_interface/robot_hw.h>
#include <bitbots_ros_control/utils.h>

#include <dynamixel_workbench/dynamixel_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

namespace bitbots_ros_control {

class CoreHardwareInterface : public hardware_interface::RobotHW {
 public:
  explicit CoreHardwareInterface(std::shared_ptr<DynamixelDriver> &driver, int id, int read_rate);

  bool init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) override;

  void read(const ros::Time &t, const ros::Duration &dt);

  void write(const ros::Time &t, const ros::Duration &dt);

 private:
  bool switch_power(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);

  ros::NodeHandle nh_;

  std::shared_ptr<DynamixelDriver> driver_;

  // always keep the lasts values to check if they different
  std::vector<std::vector<double>> current_pressure_;

  int id_;
  int read_rate_;
  int read_counter_;
  uint8_t *data_;

  bool requested_power_switch_status_;
  std_msgs::Bool power_switch_status_;
  std_msgs::Float64 VCC_;
  std_msgs::Float64 VBAT_;
  std_msgs::Float64 VEXT_;
  std_msgs::Float64 VDXL_;
  std_msgs::Float64 current_;

  ros::Publisher diagnostic_pub_;
  ros::Publisher power_pub_;
  ros::Publisher vcc_pub_;
  ros::Publisher vbat_pub_;
  ros::Publisher vext_pub_;
  ros::Publisher vdxl_pub_;
  ros::Publisher current_pub_;

  ros::ServiceServer power_switch_service_;
};
}
#endif
