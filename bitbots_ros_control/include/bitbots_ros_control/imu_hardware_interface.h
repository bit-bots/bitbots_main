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
#include <std_srvs/Empty.h>

namespace bitbots_ros_control
{

class ImuHardwareInterface : public hardware_interface::RobotHW{
public:
  ImuHardwareInterface();
  explicit ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, uint8_t id);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  uint8_t id_;
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

  bool calibrate_gyro_ = false; 
  bool reset_gyro_calibration_ = false;

  bool write_complementary_filter_params_ = false;
  bool do_adaptive_gain_, do_bias_estimation_;
  float accel_gain_, bias_alpha_;

  ros::ServiceServer imu_ranges_service_, calibrate_gyro_service_, reset_gyro_calibration_service_,
                     complementary_filter_params_service_;

  bool setIMURanges(bitbots_msgs::IMURangesRequest& req, bitbots_msgs::IMURangesResponse& resp);
  bool calibrateGyro(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool resetGyroCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool setComplementaryFilterParams(bitbots_msgs::ComplementaryFilterParamsRequest& req,
                                    bitbots_msgs::ComplementaryFilterParamsResponse& resp);
};
}
#endif