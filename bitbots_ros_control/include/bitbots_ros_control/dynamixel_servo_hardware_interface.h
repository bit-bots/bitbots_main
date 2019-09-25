#ifndef DYNAMIXEL_HARWARE_INTERFACE_H
#define DYNAMIXEL_HARWARE_INTERFACE_H

#include <ros/ros.h>
#include <string> 

#include <std_msgs/Bool.h>  
#include <humanoid_league_msgs/Speak.h>
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

#include <bitbots_ros_control/dynamixel_servo_hardware_interface_paramsConfig.h>

#include <dynamixel_workbench/dynamixel_driver.h>
#include <bitset>

namespace bitbots_ros_control
{
template<typename T>
std::string vecToString(const std::vector<T>& vec)
{
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct State
{
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

struct Joint
{
  std::string name;
  State current;
  State goal;
};

enum ControlMode {
  PositionControl,
  VelocityControl,
  EffortControl,
  CurrentBasedPositionControl
};

class DynamixelServoHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelServoHardwareInterface();
  DynamixelServoHardwareInterface(std::shared_ptr<DynamixelDriver>& driver);
  void reconf_callback(bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig &config, uint32_t level);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle _nh;

  void syncWritePWM();

  bool loadDynamixels(ros::NodeHandle& nh);
  bool writeROMRAM(ros::NodeHandle& nh);
  bool stringToControlMode(std::string control_mode_str, ControlMode &control_mode);
  void switchDynamixelControlMode();
  diagnostic_msgs::DiagnosticStatus createServoDiagMsg(int id, char level, std::string message, std::map<std::string, std::string> map);
  void processVTE(bool success);

  bool goal_torque_;
  bool current_torque_;
  void writeTorque(bool enabled);
  void setTorqueCb(std_msgs::BoolConstPtr enabled);
  void writeTorqueForServos(std::vector<int32_t> torque);
  void individualTorqueCb(bitbots_msgs::JointTorque msg);

  bool syncReadPositions();
  bool syncReadVelocities();
  bool syncReadEfforts();
  bool syncReadAll();
  bool syncReadVoltageAndTemp();
  bool syncReadError();

  void syncWritePosition();
  void syncWriteVelocity();
  void syncWriteProfileVelocity();
  void syncWriteCurrent();
  void syncWriteProfileAcceleration();

  bool first_cycle_;
  bool _lost_servo_connection;
  
  bool _switch_individual_torque;
  std::vector<int32_t> _goal_torque_individual;

  std::shared_ptr<DynamixelDriver> _driver;

  hardware_interface::JointStateInterface _jnt_state_interface;
  hardware_interface::PositionJointInterface _jnt_pos_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  hardware_interface::EffortJointInterface _jnt_eff_interface;
  hardware_interface::PosVelAccCurJointInterface _jnt_posvelacccur_interface;

  hardware_interface::RobotHW* _parent;

  ControlMode _control_mode;

  int _joint_count;

  std::vector<std::string> _joint_names;
  std::vector<uint8_t> _joint_ids;
  std::vector<double> _joint_mounting_offsets;
  std::vector<double> _joint_offsets;

  std::vector<double> _goal_position;
  std::vector<double> _goal_effort;
  std::vector<double> _goal_velocity;
  std::vector<double> _goal_acceleration;

  std::vector<double> _last_goal_position;
  std::vector<double> _last_goal_effort;
  std::vector<double> _last_goal_velocity;
  std::vector<double> _last_goal_acceleration;

  bool _read_position;
  bool _read_velocity;
  bool _read_effort;
  bool _read_volt_temp;
  std::vector<double> _current_position;
  std::vector<double> _current_velocity;
  std::vector<double> _current_effort;
  std::vector<double> _current_input_voltage;
  std::vector<double> _current_temperature;
  std::vector<uint8_t> _current_error;

  int _read_VT_counter;
  int _VT_update_rate;
  double _warn_temp;
  double _warn_volt;

  bool _torquelessMode;
  bool _onlySensors;

  int _reading_errors;
  int _reading_successes;

  // subscriber / publisher
  ros::Subscriber _set_torque_sub;
  ros::Publisher _diagnostic_pub;
  ros::Publisher _speak_pub;
  ros::Subscriber _set_torque_indiv_sub;

  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> *_dyn_reconf_server;

};
}

#endif
