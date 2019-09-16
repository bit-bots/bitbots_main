#ifndef IMU_HARWARE_INTERFACE_H
#define IMU_HARWARE_INTERFACE_H

#include <ros/ros.h>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

#include <bitbots_ros_control/bitbots_ros_control_paramsConfig.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace bitbots_ros_control
{

class ImuHardwareInterface : public hardware_interface::RobotHW{
public:
  ImuHardwareInterface();
  ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle _nh;
  std::shared_ptr<DynamixelDriver> _driver;
  hardware_interface::ImuSensorInterface _imu_interface;
  hardware_interface::RobotHW* _parent;

  uint32_t _last_seq_number{};
  double* _orientation{}; //quaternion (x,y,z,w)
  double* _orientation_covariance{};
  double* _angular_velocity{};
  double* _angular_velocity_covariance{};
  double* _linear_acceleration{};
  double* _linear_acceleration_covariance{};

  diagnostic_msgs::DiagnosticStatus _status_IMU;


};
}
#endif