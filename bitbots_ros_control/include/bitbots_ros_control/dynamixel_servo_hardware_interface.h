#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <string> 

#include <std_msgs/Bool.h>  
#include <humanoid_league_msgs/Speak.h>
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
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  EFFORT_CONTROL,
  CURRENT_BASED_POSITION_CONTROL
};

class DynamixelServoHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelServoHardwareInterface();
  explicit DynamixelServoHardwareInterface(std::shared_ptr<DynamixelDriver>& driver);
  void reconfCallback(bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig &config, uint32_t level);

  bool init(ros::NodeHandle& nh);
  bool read();
  void write();
  void setParent(hardware_interface::RobotHW* parent);

private:
  ros::NodeHandle nh_;

  void syncWritePWM();

  bool loadDynamixels(ros::NodeHandle& nh);
  bool writeROMRAM(ros::NodeHandle& nh);
  bool stringToControlMode(std::string control_mode_str, ControlMode &control_mode);
  void switchDynamixelControlMode();
  diagnostic_msgs::DiagnosticStatus createServoDiagMsg(int id, char level, std::string message, std::map<std::string, std::string> map);
  void processVte(bool success);

  bool goal_torque_;
  bool current_torque_;
  void writeTorque(bool enabled);
  void setTorqueCb(std_msgs::BoolConstPtr enabled);
  void writeTorqueForServos(std::vector<int32_t> torque);
  void individualTorqueCb(bitbots_msgs::JointTorque msg);

  bool syncReadPositions();
  bool syncReadVelocities();
  bool syncReadEfforts();
  bool syncReadPWMs();
  bool syncReadAll();
  bool syncReadVoltageAndTemp();
  bool syncReadError();

  void syncWritePosition();
  void syncWriteVelocity();
  void syncWriteProfileVelocity();
  void syncWriteCurrent();
  void syncWriteProfileAcceleration();

  bool first_cycle_;
  bool lost_servo_connection_;
  
  bool switch_individual_torque_;
  std::vector<int32_t> goal_torque_individual_;

  std::shared_ptr<DynamixelDriver> driver_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;
  hardware_interface::PosVelAccCurJointInterface jnt_posvelacccur_interface_;

  hardware_interface::RobotHW* parent_;

  ControlMode control_mode_;

  int joint_count_;

  std::vector<std::string> joint_names_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> joint_mounting_offsets_;
  std::vector<double> joint_offsets_;

  std::vector<double> goal_position_;
  std::vector<double> goal_effort_;
  std::vector<double> goal_velocity_;
  std::vector<double> goal_acceleration_;

  std::vector<double> last_goal_position_;
  std::vector<double> last_goal_effort_;
  std::vector<double> last_goal_velocity_;
  std::vector<double> last_goal_acceleration_;

  bool read_position_;
  bool read_velocity_;
  bool read_effort_;
  bool read_PWM_;
  bool read_volt_temp_;
  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  std::vector<double> current_effort_;
  std::vector<double> current_pwm_;
  std::vector<double> current_input_voltage_;
  std::vector<double> current_temperature_;
  std::vector<uint8_t> current_error_;

  int read_vt_counter_;
  int vt_update_rate_;
  double warn_temp_;
  double warn_volt_;

  bool torqueless_mode_;

  int reading_errors_;
  int reading_successes_;

  // subscriber / publisher
  ros::Subscriber set_torque_sub_;
  ros::Publisher pwm_pub_;
  ros::Publisher diagnostic_pub_;
  ros::Publisher speak_pub_;
  ros::Subscriber set_torque_indiv_sub_;

  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> *dyn_reconf_server_;

};
}

#endif
