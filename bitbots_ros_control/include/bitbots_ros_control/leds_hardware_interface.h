#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_LEDS_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_LEDS_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>


#include <hardware_interface/robot_hw.h>

#include <dynamixel_workbench/dynamixel_driver.h>

#include <bitbots_msgs/Leds.h>

namespace bitbots_ros_control
{

class LedsHardwareInterface : public hardware_interface::RobotHW{
public:
  LedsHardwareInterface();
  explicit LedsHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, uint8_t id, uint8_t num_leds);

  bool init(ros::NodeHandle& nh, ros::NodeHandle &hw_nh);
  void read(const ros::Time& t, const ros::Duration& dt);
  void write(const ros::Time& t, const ros::Duration& dt);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  uint8_t id_;

  bool write_leds_ = false;
  std::vector<uint32_t> leds_;

  ros::ServiceServer leds_service_;
  bool setLeds(bitbots_msgs::LedsRequest& req, bitbots_msgs::LedsResponse& resp);
};
}
#endif