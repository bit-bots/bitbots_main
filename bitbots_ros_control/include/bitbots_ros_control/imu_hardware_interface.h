#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <dynamixel_driver.h>

#include <bitbots_msgs/srv/imu_ranges.hpp>
#include <bitbots_msgs/srv/complementary_filter_params.hpp>
#include <bitbots_msgs/srv/accelerometer_calibration.hpp>
#include <bitbots_msgs/srv/set_accelerometer_calibration_threshold.hpp>
#include <std_srvs/srv/empty.hpp>
#include <bitbots_ros_control/hardware_interface.h>

namespace bitbots_ros_control {

class ImuHardwareInterface : public bitbots_ros_control::HardwareInterface{
 public:
  explicit ImuHardwareInterface(rclcpp::Node::SharedPtr nh,
                                std::shared_ptr<DynamixelDriver> &driver,
                                int id,
                                std::string topic,
                                std::string frame,
                                std::string name);

  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);

 private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  int id_;
  std::string topic_;
  std::string frame_;
  std::string name_;
  uint8_t *data_;
  uint8_t *accel_calib_data_;

  uint32_t last_seq_number_{};
  double *orientation_{}; //quaternion (x,y,z,w)
  double *orientation_covariance_{};
  double *angular_velocity_{};
  double *angular_velocity_covariance_{};
  double *linear_acceleration_{};
  double *linear_acceleration_covariance_{};

  diagnostic_msgs::msg::DiagnosticStatus status_imu_;

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

  rclcpp::Service<bitbots_msgs::srv::IMURanges>::SharedPtr imu_ranges_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibrate_gyro_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_gyro_calibration_service_;
  rclcpp::Service<bitbots_msgs::srv::ComplementaryFilterParams>::SharedPtr complementary_filter_params_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibrate_accel_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_accel_calibration_service_;
  rclcpp::Service<bitbots_msgs::srv::AccelerometerCalibration>::SharedPtr read_accel_calibration_service_;
  rclcpp::Service<bitbots_msgs::srv::SetAccelerometerCalibrationThreshold>::SharedPtr set_accel_calib_threshold_service_;

  void setIMURanges(const std::shared_ptr<bitbots_msgs::srv::IMURanges::Request> req, std::shared_ptr<bitbots_msgs::srv::IMURanges::Response> resp);
  void calibrateGyro(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  void resetGyroCalibration(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  void setComplementaryFilterParams(const std::shared_ptr<bitbots_msgs::srv::ComplementaryFilterParams::Request> req,
                                    std::shared_ptr<bitbots_msgs::srv::ComplementaryFilterParams::Response> resp);
  void calibrateAccel(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  void resetAccelCalibraton(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  void readAccelCalibration(const std::shared_ptr<bitbots_msgs::srv::AccelerometerCalibration::Request> req,
                            std::shared_ptr<bitbots_msgs::srv::AccelerometerCalibration::Response> resp);
  void setAccelCalibrationThreshold(const std::shared_ptr<bitbots_msgs::srv::SetAccelerometerCalibrationThreshold::Request> req,
                                    std::shared_ptr<bitbots_msgs::srv::SetAccelerometerCalibrationThreshold::Response> resp);
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  int diag_counter_;
};
}
#endif