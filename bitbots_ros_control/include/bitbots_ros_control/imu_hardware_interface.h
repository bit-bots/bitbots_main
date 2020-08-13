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
#include <bitbots_msgs/AccelerometerCalibration.h>
#include <bitbots_msgs/SetAccelerometerCalibrationThreshold.h>
#include <std_srvs/Empty.h>


namespace bitbots_ros_control
{

class ImuHardwareInterface : public hardware_interface::RobotHW{
public:
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
  uint8_t *data_;
  uint8_t *accel_calib_data_;

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

  bool calibrate_accel_ = false;
  bool reset_accel_calibration_ = false;

  bool read_accel_calibration_ = false;
  float accel_calib_threshold_read_;
  float accel_calib_bias_[3];
  float accel_calib_scale_[3];
  
  bool set_accel_calib_threshold_ = false;
  float accel_calib_threshold_;

  ros::ServiceServer imu_ranges_service_, calibrate_gyro_service_, reset_gyro_calibration_service_,
                     complementary_filter_params_service_, calibrate_accel_service_, reset_accel_calibration_service_,
                     read_accel_calibration_service_, set_accel_calib_threshold_service_;

  bool setIMURanges(bitbots_msgs::IMURangesRequest& req, bitbots_msgs::IMURangesResponse& resp);
  bool calibrateGyro(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool resetGyroCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool setComplementaryFilterParams(bitbots_msgs::ComplementaryFilterParamsRequest& req,
                                    bitbots_msgs::ComplementaryFilterParamsResponse& resp);
  bool calibrateAccel(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool resetAccelCalibraton(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
  bool readAccelCalibration(bitbots_msgs::AccelerometerCalibrationRequest& req, bitbots_msgs::AccelerometerCalibrationResponse& resp);
  bool setAccelCalibrationThreshold(bitbots_msgs::SetAccelerometerCalibrationThresholdRequest& req,
                                    bitbots_msgs::SetAccelerometerCalibrationThresholdResponse& resp);
};
}
#endif