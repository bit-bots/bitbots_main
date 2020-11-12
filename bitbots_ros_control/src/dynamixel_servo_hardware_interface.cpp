#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

#include <utility>

namespace bitbots_ros_control {

DynamixelServoHardwareInterface::DynamixelServoHardwareInterface() {}


void DynamixelServoHardwareInterface::addBusInterface(ServoBusInterface *bus) {
  bus_interfaces_.push_back(bus);
}

bool DynamixelServoHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) {
  /*
  * This initializes the hardware interface based on the values set in the config.
  * The servos are pinged to verify that a connection is present and to know which type of servo it is.
  */
  nh_ = nh;

  // Init subscriber / publisher
  set_torque_sub_ = nh.subscribe<std_msgs::BoolConstPtr>("set_torque",
                                                         1,
                                                         &DynamixelServoHardwareInterface::setTorqueCb,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
  set_torque_indiv_sub_ = nh.subscribe<bitbots_msgs::JointTorque>("set_torque_individual",
                                                                  1,
                                                                  &DynamixelServoHardwareInterface::individualTorqueCb,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());
  pwm_pub_ = nh.advertise<sensor_msgs::JointState>("/servo_PWM", 10, true);

  torqueless_mode_ = nh.param("torqueless_mode", false);
  // init merged vectors for controller
  joint_count_ = 0;
  for (ServoBusInterface *bus: bus_interfaces_) {
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

  // register ros_control interfaces
  for (unsigned int i = 0; i < joint_names_.size(); i++) {
    hardware_interface::JointStateHandle
        state_handle(joint_names_[i], &current_position_[i], &current_velocity_[i], &current_effort_[i]);
    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &goal_position_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &goal_velocity_[i]);
    jnt_vel_interface_.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &goal_effort_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);

    hardware_interface::PosVelAccCurJointHandle posvelacccur_handle
        (state_handle, &goal_position_[i], &goal_velocity_[i], &goal_acceleration_[i], &goal_effort_[i]);
    jnt_posvelacccur_interface_.registerHandle(posvelacccur_handle);

  }
  parent_->registerInterface(&jnt_state_interface_);

  std::string control_mode;
  nh.getParam("servos/control_mode", control_mode);
  ROS_INFO("Control mode: %s", control_mode.c_str());
  if (!stringToControlMode(control_mode, control_mode_)) {
    ROS_ERROR_STREAM("Unknown control mode'" << control_mode << "'.");
    return false;
  }
  if (control_mode_ == POSITION_CONTROL) {
    // we use the posvelacccur interface to be compatible to the rest of our software
    // normally this should be a position interface
    parent_->registerInterface(&jnt_posvelacccur_interface_);
  } else if (control_mode_ == VELOCITY_CONTROL) {
    parent_->registerInterface(&jnt_vel_interface_);
  } else if (control_mode_ == EFFORT_CONTROL) {
    parent_->registerInterface(&jnt_eff_interface_);
  } else if (control_mode_ == CURRENT_BASED_POSITION_CONTROL) {
    parent_->registerInterface(&jnt_posvelacccur_interface_);
  }


  // init dynamic reconfigure
  dyn_reconf_server_ =
      new dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>(ros::NodeHandle(
          "~/servos"));
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconfCallback, this, _1, _2);
  dyn_reconf_server_->setCallback(f);

  pwm_msg_ = sensor_msgs::JointState();
  pwm_msg_.name = joint_names_;

  ROS_INFO("Hardware interface init finished.");
  return true;
}

void DynamixelServoHardwareInterface::individualTorqueCb(bitbots_msgs::JointTorque msg) {
  /**
   * Handles incomming JointTroque messages and remembers the requested torque configuration of the servos.
   * It will not directly written since this has to happen in the main write loop
   */
  if (torqueless_mode_) {
    return;
  }

  // we save the goal torque value. It will be set during write process
  for (int i = 0; i < msg.joint_names.size(); i++) {
    bool success = false;
    for (int j = 0; j < joint_names_.size(); j++) {
      if (msg.joint_names[i] == joint_names_[j]) {
        if (i < msg.joint_names.size()) {
          goal_torque_individual_[j] = msg.on[i];
          success = true;
        } else {
          ROS_WARN("Somethings wrong with your message to set torques.");
        }
      }
    }
    if (!success) {
      ROS_WARN("Couldn't set torque for servo %s ", msg.joint_names[i].c_str());
    }
  }
  for (ServoBusInterface *bus: bus_interfaces_) {
    bus->switch_individual_torque_ = true;
  }
}

void DynamixelServoHardwareInterface::setTorqueCb(std_msgs::BoolConstPtr enabled) {
  /**
   * This saves the given required value, so that it can be written to the servos in the write method
   */
  for (ServoBusInterface *bus: bus_interfaces_) {
    bus->goal_torque_ = enabled->data;
  }
  for (int j = 0; j < joint_names_.size(); j++) {
    goal_torque_individual_[j] = enabled->data;
  }
}

void DynamixelServoHardwareInterface::setParent(hardware_interface::RobotHW *parent) {
  /**
   * We need the parent to be able to register the controller interface.
   */
  parent_ = parent;
}

void DynamixelServoHardwareInterface::reconfCallback(bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig &config,
                                                     uint32_t level) {
  // set values on every bus
  for (ServoBusInterface *bus: bus_interfaces_) {
    bus->read_position_ = config.read_position;
    bus->read_velocity_ = config.read_velocity;
    bus->read_effort_ = config.read_effort;
    bus->read_pwm_ = config.read_pwm;
    bus->read_volt_temp_ = config.read_volt_temp;
    bus->vt_update_rate_ = config.VT_update_rate;
    bus->warn_temp_ = config.warn_temp;
    bus->warn_volt_ = config.warn_volt;
  }
}

void DynamixelServoHardwareInterface::read(const ros::Time &t, const ros::Duration &dt) {
  // retrieve values from the buses and set controller vector accordingly
  //todo improve performance
  int i = 0;
  for (ServoBusInterface *bus: bus_interfaces_) {
    for (int j = 0; j < bus->joint_count_; j++) {
      current_position_[i] = bus->current_position_[j];
      current_velocity_[i] = bus->current_velocity_[j];
      current_effort_[i] = bus->current_effort_[j];
      current_pwm_[i] = bus->current_pwm_[j];
      i++;
    }
  }
  // PWM values are not part of joint state controller and have to be published independently
  pwm_msg_.header.stamp = ros::Time::now();
  pwm_msg_.effort = current_pwm_;
  pwm_pub_.publish(pwm_msg_);
}

void DynamixelServoHardwareInterface::write(const ros::Time &t, const ros::Duration &dt) {
  // set all values from controller to the buses
  //todo improve performance
  int i = 0;
  for (ServoBusInterface *bus: bus_interfaces_) {
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
}
