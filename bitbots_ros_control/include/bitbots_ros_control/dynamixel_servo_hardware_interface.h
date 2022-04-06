#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <std_msgs/msg/bool.hpp>
#include <humanoid_league_msgs/msg/audio.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <bitbots_msgs/msg/joint_torque.hpp>

#include <bitbots_ros_control/utils.h>

#include <bitbots_ros_control/servo_bus_interface.h>
#include <dynamixel_driver.h>
#include <bitset>

namespace bitbots_ros_control {
template<typename T>
std::string vecToString(const std::vector<T> &vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct State {
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

struct Joint {
  std::string name;
  State current;
  State goal;
};

class DynamixelServoHardwareInterface{
 public:
  explicit DynamixelServoHardwareInterface(rclcpp::Node::SharedPtr nh);

  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void addBusInterface(ServoBusInterface *bus);

 private:
  rclcpp::Node::SharedPtr nh_;
  std::vector<ServoBusInterface *> bus_interfaces_;

  void setTorqueCb(std_msgs::msg::Bool::SharedPtr enabled);
  void individualTorqueCb(bitbots_msgs::msg::JointTorque msg);

  std::vector<int32_t> goal_torque_individual_;

  ControlMode control_mode_;

  int joint_count_;

  std::vector<std::string> joint_names_;

  std::vector<double> goal_position_;
  std::vector<double> goal_effort_;
  std::vector<double> goal_velocity_;
  std::vector<double> goal_acceleration_;

  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  std::vector<double> current_effort_;
  std::vector<double> current_pwm_;
  std::vector<double> current_input_voltage_;
  std::vector<double> current_temperature_;
  std::vector<uint8_t> current_error_;

  bool torqueless_mode_;

  // subscriber / publisher
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_torque_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pwm_pub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointTorque>::SharedPtr set_torque_indiv_sub_;

  sensor_msgs::msg::JointState pwm_msg_;
};
}

#endif