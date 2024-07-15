#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_

#include <dynamixel_driver.h>

#include <bitbots_msgs/msg/audio.hpp>
#include <bitbots_msgs/msg/joint_torque.hpp>
#include <bitbots_ros_control/hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>
#include <bitset>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>

namespace bitbots_ros_control {

class ServoBusInterface : public bitbots_ros_control::HardwareInterface {
 public:
  explicit ServoBusInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver,
                             std::vector<std::tuple<int, std::string, float, float, std::string>> servos);
  ~ServoBusInterface();
  bool init();
  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);
  void restoreAfterPowerCycle();

  bool loadDynamixels();
  bool writeROMRAM(bool first_time);

  void switchDynamixelControlMode();
  diagnostic_msgs::msg::DiagnosticStatus createServoDiagMsg(int id, char level, std::string message,
                                                            std::map<std::string, std::string> map, std::string name);
  void processVte(bool success);

  bool goal_torque_;
  bool current_torque_;
  void writeTorque(bool enabled);
  void writeTorqueForServos(std::vector<int32_t> torque);

  bool readPositions();
  bool singleReadPositions();
  bool syncReadPositions();
  bool syncReadVelocities();
  bool syncReadEfforts();
  bool syncReadPWMs();
  bool syncReadAll();
  bool readVoltageAndTemp();
  bool singleReadVoltageAndTemp();
  bool syncReadVoltageAndTemp();
  bool readError();
  bool singleReadError();
  bool syncReadError();

  void writePosition();
  void syncWritePosition();
  void singleWritePosition();
  void syncWriteVelocity();
  void writeProfileVelocity();
  void singleWriteProfileVelocity();
  void syncWriteProfileVelocity();
  void syncWriteCurrent();
  void writeProfileAcceleration();
  void singleWriteProfileAcceleration();
  void syncWriteProfileAcceleration();
  void writePWM();
  void singleWritePWM();
  void syncWritePWM();

  void disableSyncInstructions();
  bool disable_sync_instructions_;

  rclcpp::Node::SharedPtr nh_;
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
  // id, name, modelnumber, group
  std::vector<std::tuple<int, std::string, float, float, std::string>> servos_;
  int joint_count_;

  std::vector<int32_t> goal_torque_individual_;

  std::vector<std::string> joint_names_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> joint_mounting_offsets_;
  std::vector<double> joint_offsets_;
  std::vector<std::string> joint_groups_;  // The group name for each joint

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
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<bitbots_msgs::msg::Audio>::SharedPtr speak_pub_;
};
}  // namespace bitbots_ros_control
#endif  // BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_SERVO_BUS_INTERFACE_H_
