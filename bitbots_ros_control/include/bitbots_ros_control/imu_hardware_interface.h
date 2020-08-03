#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace bitbots_ros_control
{

class ImuHardwareInterface : public hardware_interface::RobotHW{
public:
  ImuHardwareInterface();
  explicit ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, int id, std::string topic, std::string frame, std::string name);

  bool init(ros::NodeHandle& nh, ros::NodeHandle &hw_nh);
  void read(const ros::Time& t, const ros::Duration& dt);
  void write(const ros::Time& t, const ros::Duration& dt);
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  int id_;
  std::string topic_;
  std::string frame_;
  std::string name_;
  hardware_interface::ImuSensorInterface imu_interface_;
  hardware_interface::RobotHW* parent_;

  uint32_t last_seq_number_{};
  double* orientation_{}; //quaternion (x,y,z,w)
  double* orientation_covariance_{};
  double* angular_velocity_{};
  double* angular_velocity_covariance_{};
  double* linear_acceleration_{};
  double* linear_acceleration_covariance_{};

  diagnostic_msgs::DiagnosticStatus status_imu_;


};
}
#endif