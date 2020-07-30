#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dynamixel_workbench/dynamixel_driver.h>

#include <bitbots_msgs/IMURanges.h>
#include <bitbots_msgs/ComplementaryFilterParams.h>

namespace bitbots_ros_control
{

class ImuHardwareInterface : public hardware_interface::RobotHW{
public:
  ImuHardwareInterface();
  explicit ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
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

  bool write_ranges_ = false;
  uint8_t gyro_range_, accel_range_;

  bool write_complementary_filter_params_ = false;
  bool do_adaptive_gain_, do_bias_estimation_;
  float accel_gain_, bias_alpha_;
  ros::ServiceServer imu_ranges_service_, calibrate_gyro_service_, reset_gyro_calibration_service_,
                     complementary_filter_params_service_;

  bool setIMURanges(bitbots_msgs::IMURangesRequest& req, bitbots_msgs::IMURangesResponse& resp);
};
}
#endif