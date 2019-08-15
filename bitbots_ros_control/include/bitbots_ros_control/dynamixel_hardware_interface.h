#ifndef DYNAMIXEL_HARWARE_INTERFACE_H
#define DYNAMIXEL_HARWARE_INTERFACE_H

#include <ros/ros.h>
#include <string> 

#include <std_msgs/Bool.h>  
#include <humanoid_league_msgs/Speak.h>
#include <bitbots_buttons/Buttons.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <bitbots_msgs/JointTorque.h>
#include <bitbots_msgs/FootPressure.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <bitbots_ros_control/posvelacccur_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <dynamic_reconfigure/server.h>

#include <bitbots_ros_control/bitbots_ros_control_paramsConfig.h>

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

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface();
  void reconf_callback(bitbots_ros_control::bitbots_ros_control_paramsConfig &config, uint32_t level);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();

private:
  ros::NodeHandle _nh;

  bool syncWritePWM();

  bool loadDynamixels(ros::NodeHandle& nh);
  bool writeROMRAM(ros::NodeHandle& nh);
  bool stringToControlMode(std::string control_mode_str, ControlMode &control_mode);
  bool switchDynamixelControlMode();
  diagnostic_msgs::DiagnosticStatus createServoDiagMsg(int id, char level, std::string message, std::map<std::string, std::string> map);
  void processVTE(bool success);

  void speak(std::string text);

  bool goal_torque_;
  bool current_torque_;
  void writeTorque(bool enabled);
  void setTorque(std_msgs::BoolConstPtr enabled);
  void writeTorqueForServos(std::vector<int32_t> torque);
  void individualTorqueCb(bitbots_msgs::JointTorque msg);


  bool syncReadPositions();
  bool syncReadVelocities();
  bool syncReadEfforts();
  bool syncReadAll();
  bool syncReadVoltageAndTemp();
  bool syncReadError();
  bool readImu();
  bool readButtons();
  bool readFootSensors();


  bool syncWritePosition();
  bool syncWriteVelocity();
  bool syncWriteProfileVelocity();
  bool syncWriteCurrent();
  bool syncWriteProfileAcceleration();



  bool first_cycle_;
  bool _lost_servo_connection;
  
  bool _switch_individual_torque;
  std::vector<int32_t> _goal_torque_individual;

  boost::shared_ptr<DynamixelDriver> _driver;

  hardware_interface::JointStateInterface _jnt_state_interface;

  hardware_interface::PositionJointInterface _jnt_pos_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  hardware_interface::EffortJointInterface _jnt_eff_interface;
  hardware_interface::PosVelAccCurJointInterface _jnt_posvelacccur_interface;

  hardware_interface::ImuSensorInterface _imu_interface;

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

  std::vector<double> _current_pressure;

  int _read_VT_counter;
  int _VT_update_rate;
  double _warn_temp;
  double _warn_volt;

  bool _torquelessMode;
  bool _read_imu;
  bool _read_pressure;
  bool _readButtons;
  bool _onlySensors;  
  uint32_t _last_seq_number;
  double* _orientation; //quaternion (x,y,z,w)
  double* _orientation_covariance;
  double* _angular_velocity;
  double* _angular_velocity_covariance;
  double* _linear_acceleration;
  double* _linear_acceleration_covariance;

  int _reading_errors;
  int _reading_successes;

  diagnostic_msgs::DiagnosticStatus _status_board;
  diagnostic_msgs::DiagnosticStatus _status_IMU;
  diagnostic_msgs::DiagnosticStatus _status_servo;
  // subscriber / publisher
  ros::Subscriber _set_torque_sub;
  ros::Publisher _diagnostic_pub;
  ros::Publisher _speak_pub;
  ros::Publisher _button_pub;
  ros::Publisher _pressure_pub;
  ros::Subscriber _set_torque_indiv_sub;
  ros::Subscriber _update_pid_sub;


};
}

#endif
