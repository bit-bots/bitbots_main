#include <bitbots_ros_control/dynamixel_servo_hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>
#include <utility>

namespace bitbots_ros_control {
using std::placeholders::_1;

DynamixelServoHardwareInterface::DynamixelServoHardwareInterface(rclcpp::Node::SharedPtr nh) { nh_ = nh; }

void DynamixelServoHardwareInterface::addBusInterface(ServoBusInterface *bus) { bus_interfaces_.push_back(bus); }

void DynamixelServoHardwareInterface::writeROMRAM(bool first_time) {
  for (ServoBusInterface *bus : bus_interfaces_) {
    bus->writeROMRAM(first_time);
  }
}

bool DynamixelServoHardwareInterface::init() {
  /*
   * This initializes the hardware interface based on the values set in the config.
   * The servos are pinged to verify that a connection is present and to know which type of servo it is.
   */

  // Init subscriber / publisher
  set_torque_sub_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "set_torque", 1, std::bind(&DynamixelServoHardwareInterface::setTorqueCb, this, _1));
  set_torque_indiv_sub_ = nh_->create_subscription<bitbots_msgs::msg::JointTorque>(
      "set_torque_individual", 1, std::bind(&DynamixelServoHardwareInterface::individualTorqueCb, this, _1));
  // todo we could change the command topic to something better
  sub_command_ = nh_->create_subscription<bitbots_msgs::msg::JointCommand>(
      "/DynamixelController/command", 1,
      std::bind(&DynamixelServoHardwareInterface::commandCb, this, std::placeholders::_1));
  pwm_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("/servo_PWM", 10);
  joint_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  torqueless_mode_ = nh_->get_parameter("torqueless_mode").as_bool();

  // init merged vectors for controller
  joint_count_ = 0;
  for (ServoBusInterface *bus : bus_interfaces_) {
    joint_count_ = joint_count_ + bus->joint_count_;
    for (int i = 0; i < bus->joint_count_; i++) {
      joint_names_.push_back(bus->joint_names_[i]);
    }
  }
  current_position_.resize(joint_count_, 0);
  current_velocity_.resize(joint_count_, 0);
  current_effort_.resize(joint_count_, 0);
  current_pwm_.resize(joint_count_, 0);
  current_input_voltage_.resize(joint_count_, 0);
  current_temperature_.resize(joint_count_, 0);
  current_error_.resize(joint_count_, 0);
  goal_position_.resize(joint_count_, 0);
  goal_velocity_.resize(joint_count_, 0);
  goal_acceleration_.resize(joint_count_, 0);
  goal_effort_.resize(joint_count_, 0);
  goal_torque_individual_.resize(joint_count_, 1);

  // create mapping for faster computation later
  for (unsigned int i = 0; i < joint_count_; i++) {
    joint_map_[joint_names_[i]] = i;
  }

  std::string control_mode;
  control_mode = nh_->get_parameter("servos.control_mode").as_string();
  RCLCPP_INFO(nh_->get_logger(), "Control mode: %s", control_mode.c_str());
  if (!stringToControlMode(nh_, control_mode, control_mode_)) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Unknown control mode'" << control_mode << "'.");
    return false;
  }

  joint_state_msg_ = sensor_msgs::msg::JointState();
  joint_state_msg_.name = joint_names_;
  pwm_msg_ = sensor_msgs::msg::JointState();
  pwm_msg_.name = joint_names_;

  RCLCPP_INFO(nh_->get_logger(), "Hardware interface init finished.");
  return true;
}

void DynamixelServoHardwareInterface::commandCb(const bitbots_msgs::msg::JointCommand &command_msg) {
  if (!(command_msg.joint_names.size() == command_msg.positions.size() &&
        command_msg.joint_names.size() == command_msg.velocities.size() &&
        command_msg.joint_names.size() == command_msg.accelerations.size() &&
        command_msg.joint_names.size() == command_msg.max_currents.size())) {
    RCLCPP_ERROR(nh_->get_logger(), "Dynamixel Controller got command with inconsistent array lengths.");
    return;
  }
  // todo improve performance
  for (unsigned int i = 0; i < command_msg.joint_names.size(); i++) {
    int joint_id = joint_map_[command_msg.joint_names[i]];
    goal_position_[joint_id] = command_msg.positions[i];
    goal_velocity_[joint_id] = command_msg.velocities[i];
    goal_acceleration_[joint_id] = command_msg.accelerations[i];
    goal_effort_[joint_id] = command_msg.max_currents[i];
  }
}

void DynamixelServoHardwareInterface::individualTorqueCb(bitbots_msgs::msg::JointTorque msg) {
  /**
   * Handles incomming JointTroque messages and remembers the requested torque configuration of the servos.
   * It will not directly written since this has to happen in the main write loop
   */
  if (torqueless_mode_) {
    return;
  }

  // we save the goal torque value. It will be set during write process
  for (size_t i = 0; i < msg.joint_names.size(); i++) {
    bool success = false;
    for (size_t j = 0; j < joint_names_.size(); j++) {
      if (msg.joint_names[i] == joint_names_[j]) {
        if (i < msg.joint_names.size()) {
          goal_torque_individual_[j] = msg.on[i];
          success = true;
        } else {
          RCLCPP_WARN(nh_->get_logger(), "Somethings wrong with your message to set torques.");
        }
      }
    }
    if (!success) {
      RCLCPP_WARN(nh_->get_logger(), "Couldn't set torque for servo %s ", msg.joint_names[i].c_str());
    }
  }
  for (ServoBusInterface *bus : bus_interfaces_) {
    bus->switch_individual_torque_ = true;
  }
}

void DynamixelServoHardwareInterface::setTorqueCb(std_msgs::msg::Bool::SharedPtr enabled) {
  /**
   * This saves the given required value, so that it can be written to the servos in the write method
   */
  for (ServoBusInterface *bus : bus_interfaces_) {
    bus->goal_torque_ = enabled->data;
  }
  for (size_t j = 0; j < joint_names_.size(); j++) {
    goal_torque_individual_[j] = enabled->data;
  }
}

void DynamixelServoHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  // retrieve values from the buses and set controller vector accordingly
  // todo improve performance
  int i = 0;
  for (ServoBusInterface *bus : bus_interfaces_) {
    for (int j = 0; j < bus->joint_count_; j++) {
      current_position_[i] = bus->current_position_[j];
      current_velocity_[i] = bus->current_velocity_[j];
      current_effort_[i] = bus->current_effort_[j];
      current_pwm_[i] = bus->current_pwm_[j];
      i++;
    }
  }

  // publish joint states
  joint_state_msg_.header.stamp = nh_->get_clock()->now();
  joint_state_msg_.position = current_position_;
  joint_state_msg_.velocity = current_velocity_;
  joint_state_msg_.effort = current_effort_;
  joint_pub_->publish(joint_state_msg_);

  // PWM values are not part of joint state controller and have to be published independently
  pwm_msg_.header.stamp = nh_->get_clock()->now();
  pwm_msg_.effort = current_pwm_;
  pwm_pub_->publish(pwm_msg_);
}

void DynamixelServoHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  // set all values from controller to the buses
  // todo improve performance
  int i = 0;
  for (ServoBusInterface *bus : bus_interfaces_) {
    for (int j = 0; j < bus->joint_count_; j++) {
      bus->goal_position_[j] = goal_position_[i];
      bus->goal_velocity_[j] = goal_velocity_[i];
      bus->goal_acceleration_[j] = goal_acceleration_[i];
      bus->goal_effort_[j] = goal_effort_[i];
      bus->goal_torque_individual_[j] = goal_torque_individual_[i];
      i++;
    }
  }
}
}  // namespace bitbots_ros_control
