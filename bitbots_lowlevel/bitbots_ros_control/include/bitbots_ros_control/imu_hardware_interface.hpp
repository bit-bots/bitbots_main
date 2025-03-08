#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_IMU_HARDWARE_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_msgs/srv/accelerometer_calibration.hpp>
#include <bitbots_msgs/srv/complementary_filter_params.hpp>
#include <bitbots_msgs/srv/imu_ranges.hpp>
#include <bitbots_msgs/srv/set_accelerometer_calibration_threshold.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>

namespace bitbots_ros_control {

class ImuHardwareInterface : public bitbots_ros_control::HardwareInterface {
 public:
  explicit ImuHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver, int id,
                                std::string topic, std::string frame, std::string name);

  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void restoreAfterPowerCycle();

 private:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<DynamixelDriver> driver_;
  int id_;
  std::string topic_;
  std::string frame_;
  std::string name_;
  std::array<uint8_t, 40> data_;

  uint32_t last_seq_number_{};
  std::array<double, 4> orientation_{};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 3> angular_velocity_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 3> linear_acceleration_{};
  std::array<double, 9> linear_acceleration_covariance_{};

  diagnostic_msgs::msg::DiagnosticStatus status_imu_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  sensor_msgs::msg::Imu imu_msg_;

  int diag_counter_;
};
}  // namespace bitbots_ros_control
#endif
