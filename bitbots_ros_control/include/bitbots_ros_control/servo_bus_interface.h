#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_

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

#include <dynamixel_workbench/dynamixel_driver.h>
#include <bitset>

namespace bitbots_ros_control {

class ServoBusInterface : public hardware_interface::RobotHW {
 public:
  explicit ServoBusInterface(std::shared_ptr<DynamixelDriver> &driver,
                             std::vector<std::tuple<int, std::string, float, float>> servos);
  ~ServoBusInterface();
  bool init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) override;
  void read(const ros::Time &t, const ros::Duration &dt);
  void write(const ros::Time &t, const ros::Duration &dt);

  bool loadDynamixels(ros::NodeHandle &nh);
  bool writeROMRAM(ros::NodeHandle &nh);

  void syncWritePWM();

  void switchDynamixelControlMode();
  diagnostic_msgs::DiagnosticStatus createServoDiagMsg(int id,
                                                       char level,
                                                       std::string message,
                                                       std::map<std::string, std::string> map,
                                                       std::string name);
  void processVte(bool success);

  bool goal_torque_;
  bool current_torque_;
  void writeTorque(bool enabled);
  void writeTorqueForServos(std::vector<int32_t> torque);

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

  int32_t *data_sync_read_positions_;
  int32_t *data_sync_read_velocities_;
  int32_t *data_sync_read_efforts_;
  int32_t *data_sync_read_pwms_;
  int32_t *data_sync_read_error_;
  int32_t *sync_write_goal_position_;
  int32_t *sync_write_goal_velocity_;
  int32_t *sync_write_profile_velocity_;
  int32_t *sync_write_profile_acceleration_;
  int32_t *sync_write_goal_current_;
  int32_t *sync_write_goal_pwm_;
  std::vector<uint8_t> sync_read_all_data_;

  bool first_cycle_;
  bool lost_servo_connection_;
  ControlMode control_mode_;

  bool switch_individual_torque_;

  std::shared_ptr<DynamixelDriver> driver_;
  // id, name, modelnumber
  std::vector<std::tuple<int, std::string, float, float>> servos_;
  int joint_count_;

  std::vector<int32_t> goal_torque_individual_;

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
  bool read_pwm_;
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
  ros::Publisher diagnostic_pub_;
  ros::Publisher speak_pub_;

};
}
#endif //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_
