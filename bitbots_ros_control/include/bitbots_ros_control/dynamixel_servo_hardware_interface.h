#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string>

#include <std_msgs/Bool.h>
#include <humanoid_league_msgs/Audio.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <bitbots_msgs/JointTorque.h>

#include <hardware_interface/joint_command_interface.h>
#include <bitbots_ros_control/posvelacccur_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_ros_control/utils.h>

#include <bitbots_ros_control/dynamixel_servo_hardware_interface_paramsConfig.h>

#include <bitbots_ros_control/servo_bus_interface.h>
#include <dynamixel_workbench/dynamixel_driver.h>
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

class DynamixelServoHardwareInterface : public hardware_interface::RobotHW {
 public:
  explicit DynamixelServoHardwareInterface();
  void reconfCallback(bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig &config, uint32_t level);

  bool init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) override;
  void read(const ros::Time &t, const ros::Duration &dt);
  void write(const ros::Time &t, const ros::Duration &dt);
  void addBusInterface(ServoBusInterface *bus);
  void setParent(hardware_interface::RobotHW *parent);

 private:
  ros::NodeHandle nh_;
  std::vector<ServoBusInterface *> bus_interfaces_;

  void setTorqueCb(std_msgs::BoolConstPtr enabled);
  void individualTorqueCb(bitbots_msgs::JointTorque msg);

  std::vector<int32_t> goal_torque_individual_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;
  hardware_interface::PosVelAccCurJointInterface jnt_posvelacccur_interface_;

  hardware_interface::RobotHW *parent_;

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
  ros::Subscriber set_torque_sub_;
  ros::Publisher pwm_pub_;
  ros::Subscriber set_torque_indiv_sub_;

  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> *dyn_reconf_server_;

  sensor_msgs::JointState pwm_msg_;
};
}

#endif
