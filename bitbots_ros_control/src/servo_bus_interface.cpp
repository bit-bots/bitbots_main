#include <bitbots_ros_control/servo_bus_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

ServoBusInterface::ServoBusInterface(std::shared_ptr<DynamixelDriver> &driver,
                                     std::vector<std::tuple<int, std::string, float, float>> servos)
    : first_cycle_(true), read_position_(true), read_velocity_(false), read_effort_(true) {
  driver_ = driver;
  servos_ = std::move(servos);
}

bool ServoBusInterface::init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) {
  diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  speak_pub_ = nh.advertise<humanoid_league_msgs::Audio>("/speak", 1, true);

  lost_servo_connection_ = false;
  read_vt_counter_ = 0;
  switch_individual_torque_ = false;

  torqueless_mode_ = nh.param("torqueless_mode", false);

  // Load dynamixel config from parameter server
  if (!loadDynamixels(nh)) {
    ROS_ERROR_STREAM("Failed to ping all motors.");
    speakError(speak_pub_, "Failed to ping all motors.");
    return false;
  }

  // init the different sync write and read commands that will maybe necessary
  driver_->addSyncWrite("Torque_Enable");
  driver_->addSyncWrite("Goal_Position");
  driver_->addSyncWrite("Goal_Velocity");
  driver_->addSyncWrite("Profile_Velocity");
  driver_->addSyncWrite("Profile_Acceleration");
  driver_->addSyncWrite("Goal_Current");
  driver_->addSyncWrite("Goal_PWM");
  driver_->addSyncWrite("Operating_Mode");
  driver_->addSyncRead("Present_Current");
  driver_->addSyncRead("Present_Velocity");
  driver_->addSyncRead("Present_Position");
  driver_->addSyncRead("Present_PWM");
  driver_->addSyncRead("Hardware_Error_Status");

  // Switch dynamixels to correct control mode (position, velocity, effort)
  switchDynamixelControlMode();

  // allocate correct memory for number of used joints
  joint_count_ = joint_names_.size();
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

  // malloc memory only once for later reads and writes to improve performance
  data_sync_read_positions_ = new int32_t;
  data_sync_read_velocities_ = new int32_t;
  data_sync_read_efforts_ = new int32_t;
  data_sync_read_pwms_ = new int32_t;
  data_sync_read_error_ = new int32_t;
  sync_write_goal_position_ = new int32_t;
  sync_write_goal_velocity_ = new int32_t;
  sync_write_profile_velocity_ = new int32_t;
  sync_write_profile_acceleration_ = new int32_t;
  sync_write_goal_current_ = new int32_t;
  sync_write_goal_pwm_ = new int32_t;


  // write ROM and RAM values if wanted
  if (nh.param("servos/set_ROM_RAM", false)) {
    if (!writeROMRAM(nh)) {
      ROS_WARN("Couldn't write ROM and RAM values to all servos.");
    }
  }
  writeTorque(nh.param("servos/auto_torque", false));
}

ServoBusInterface::~ServoBusInterface(){
  delete data_sync_read_positions_;
  delete data_sync_read_velocities_;
  delete data_sync_read_efforts_;
  delete data_sync_read_pwms_;
  delete data_sync_read_error_;
  delete sync_write_goal_position_;
  delete sync_write_goal_velocity_;
  delete sync_write_profile_velocity_;
  delete sync_write_profile_acceleration_;
  delete sync_write_goal_current_;
  delete sync_write_goal_pwm_;
}

bool ServoBusInterface::loadDynamixels(ros::NodeHandle &nh) {
  /**
   * Load config and try to ping servos to test if everything is correct.
   * Adds all dynamixel to the driver if they are pingable.
   */
  bool success = true;

  // prepare diagnostic msg
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();

  // get control mode
  std::string control_mode;
  nh.getParam("servos/control_mode", control_mode);
  ROS_DEBUG("Control mode: %s", control_mode.c_str());
  if (!stringToControlMode(control_mode, control_mode_)) {
    return false;
  }

  // get values to read
  nh.param("servos/read_position", read_position_, true);
  nh.param("servos/read_velocity", read_velocity_, false);
  nh.param("servos/read_effort", read_effort_, false);
  nh.param("servos/read_pwm", read_pwm_, false);

  // iterate over all servos and save the information
  // the wolfgang hardware interface already loaded them into the driver by pinging
  for (std::tuple<int, std::string, float, float> &servo : servos_) {
    int motor_id = std::get<0>(servo);
    joint_ids_.push_back(uint8_t(motor_id));
    std::string joint_name = std::get<1>(servo);
    joint_names_.push_back(joint_name);
    float mounting_offset = std::get<2>(servo);
    joint_mounting_offsets_.push_back(mounting_offset);
    float joint_offset = std::get<3>(servo);
    joint_offsets_.push_back(joint_offset);
  }

  // generate a diagnostic message
  array_msg.status = array;
  diagnostic_pub_.publish(array_msg);

  return success;
}

bool ServoBusInterface::writeROMRAM(ros::NodeHandle &nh) {
  /**
   * This method writes the ROM and RAM values specified in the config to all servos.
   */
  ROS_DEBUG("Writing ROM and RAM values");
  XmlRpc::XmlRpcValue dxls;
  nh.getParam("servos/ROM_RAM", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  bool sucess = true;
  int i = 0;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it) {
    std::string register_name = (std::string)(it->first);
    int register_value;
    nh.getParam(("servos/ROM_RAM/" + register_name).c_str(), register_value);
    ROS_DEBUG("Setting %s on all servos to %d", register_name.c_str(), register_value);

    int *values = (int *) malloc(joint_names_.size() * sizeof(int));
    for (size_t num = 0; num < joint_names_.size(); num++) {
      values[num] = register_value;
    }
    driver_->addSyncWrite(register_name.c_str());
    sucess = sucess && driver_->syncWrite(register_name.c_str(), values);
    free(values);
  }
  return sucess;
}

void ServoBusInterface::read(const ros::Time &t, const ros::Duration &dt) {
  /**
   * This is part of the main loop and handles reading of all connected devices
   */
  bool read_successful = true;
  // either read all values or a single one, depending on config
  if (read_position_ && read_velocity_ && read_effort_) {
    if (syncReadAll()) {
      for (size_t num = 0; num < joint_names_.size(); num++) {
        current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
      }
    } else {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read all current joint values!");
      read_successful = false;
    }
  } else {
    if (read_position_) {
      if (syncReadPositions()) {
        for (size_t num = 0; num < joint_names_.size(); num++) {
          current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
        }
      } else {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint position!");
        driver_->reinitSyncReadHandler("Present_Position");
        read_successful = false;
      }
    }
    if (read_velocity_) {
      if (!syncReadVelocities()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint velocity!");
        driver_->reinitSyncReadHandler("Present_Velocity");
        read_successful = false;
      }
    }
    if (read_effort_) {
      if (!syncReadEfforts()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint effort!");
        driver_->reinitSyncReadHandler("Present_Current");
        read_successful = false;
      }
    }
  }

  if (read_pwm_) {
    if (!syncReadPWMs()) {
      driver_->reinitSyncReadHandler("Present_PWM");
      ROS_ERROR_THROTTLE(1.0, "Couldn't read current PWM!");
      read_successful = false;
    }
  }

  if (read_volt_temp_) {
    if (read_vt_counter_ + 1 == vt_update_rate_) {
      bool success = true;
      if (!syncReadVoltageAndTemp()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current input volatage and temperature!");
        success = false;
      }
      if (!syncReadError()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current error bytes!");
        success = false;
        driver_->reinitSyncReadHandler("Hardware_Error_Status");
      }
      processVte(success);
    }
    read_vt_counter_ = (read_vt_counter_ + 1) % vt_update_rate_;
  }

  if (first_cycle_) {
    // when the servos have a goal position which is not the current position on startup
    // they will rapidly move to this position, possibly damaging the robot
    // therefore the goal position is set to the current position of the motors
    for (int i = 0; i < current_position_.size(); i++) {
      goal_position_[i] = current_position_[i];
    }
    first_cycle_ = false;
  }

  // remember
  if (!read_successful) {
    lost_servo_connection_ = true;
    reading_errors_ += 1;
  } else {
    reading_successes_ += 1;
  }

  if (reading_errors_ + reading_successes_ > 200 &&
      (float) reading_errors_ / (float) (reading_successes_ + reading_errors_) > 0.05f) {
    speakError(speak_pub_, "Multiple servo reading errors!");
    reading_errors_ = 0;
    reading_successes_ = 0;
  }

  if (reading_errors_ + reading_successes_ > 2000) {
    reading_errors_ = 0;
    reading_successes_ = 0;
  }
}

void ServoBusInterface::write(const ros::Time &t, const ros::Duration &dt) {
  /**
   * This is part of the mainloop and handles all the writing to the connected devices
   */
  //check if we have to switch the torque
  if (current_torque_ != goal_torque_) {
    writeTorque(goal_torque_);
  }
  if (switch_individual_torque_) {
    writeTorqueForServos(goal_torque_individual_);
    switch_individual_torque_ = false;
  }

  // reset torques if we lost connection to them
  if (lost_servo_connection_) {
    ROS_INFO_THROTTLE(5, "resetting torque after lost connection");
    writeTorqueForServos(goal_torque_individual_);
    lost_servo_connection_ = false;
  }
  if (control_mode_ == POSITION_CONTROL) {
    if (goal_effort_ != last_goal_effort_) {
      syncWritePWM();
      last_goal_effort_ = goal_effort_;
    }

    if (goal_velocity_ != last_goal_velocity_) {
      syncWriteProfileVelocity();
      last_goal_velocity_ = goal_velocity_;
    }

    if (goal_acceleration_ != last_goal_acceleration_) {
      syncWriteProfileAcceleration();
      last_goal_acceleration_ = goal_acceleration_;
    }

    if (goal_position_ != last_goal_position_) {
      syncWritePosition();
      last_goal_position_ = goal_position_;
    }
  } else if (control_mode_ == VELOCITY_CONTROL) {
    syncWriteVelocity();
  } else if (control_mode_ == EFFORT_CONTROL) {
    syncWriteCurrent();
  } else if (control_mode_ == CURRENT_BASED_POSITION_CONTROL) {
    // only write things if it is necessary
    if (goal_effort_ != last_goal_effort_) {
      syncWriteCurrent();
      last_goal_effort_ = goal_effort_;
    }

    if (goal_velocity_ != last_goal_velocity_) {
      syncWriteProfileVelocity();
      last_goal_velocity_ = goal_velocity_;
    }

    if (goal_acceleration_ != last_goal_acceleration_) {
      syncWriteProfileAcceleration();
      last_goal_acceleration_ = goal_acceleration_;
    }

    if (goal_position_ != last_goal_position_) {
      syncWritePosition();
      last_goal_position_ = goal_position_;
    }

  }
}

void ServoBusInterface::switchDynamixelControlMode() {
  /**
   * This method switches the control mode of all servos
   */

  // Torque on dynamixels has to be disabled to change operating mode
  // save last torque state for later
  bool torque_before_switch = current_torque_;
  writeTorque(false);
  // magic sleep to make sure that dynamixel have internally processed the request
  ros::Duration(0.1).sleep();

  int32_t value = 3;
  if (control_mode_ == POSITION_CONTROL) {
    value = 3;
  } else if (control_mode_ == VELOCITY_CONTROL) {
    value = 1;
  } else if (control_mode_ == EFFORT_CONTROL) {
    value = 0;
  } else if (control_mode_ == CURRENT_BASED_POSITION_CONTROL) {
    value = 5;
  } else {
    ROS_WARN("control_mode is wrong, will use position control");
  }

  std::vector<int32_t> operating_mode(joint_names_.size(), value);
  int32_t *o = &operating_mode[0];
  driver_->syncWrite("Operating_Mode", o);

  ros::Duration(0.5).sleep();
  //reenable torque if it was previously enabled
  writeTorque(torque_before_switch);
}

diagnostic_msgs::DiagnosticStatus ServoBusInterface::createServoDiagMsg(int id,
                                                                        char level,
                                                                        std::string message,
                                                                        std::map<std::string, std::string> map,
                                                                        std::string name) {
  /**
   * Create a single Diagnostic message for one servo. This is used to build the array message which is then published.
   */
  diagnostic_msgs::DiagnosticStatus servo_status = diagnostic_msgs::DiagnosticStatus();
  servo_status.level = level;
  // add prefix DS for dynamixel servo to sort in diagnostic analyser
  servo_status.name = "DS" + name;
  servo_status.message = message;
  servo_status.hardware_id = std::to_string(id);
  std::vector<diagnostic_msgs::KeyValue> keyValues = std::vector<diagnostic_msgs::KeyValue>();
  // itarate through map and save it into values
  for (auto const &ent1 : map) {
    diagnostic_msgs::KeyValue key_value = diagnostic_msgs::KeyValue();
    key_value.key = ent1.first;
    key_value.value = ent1.second;
    keyValues.push_back(key_value);
  }
  servo_status.values = keyValues;
  return servo_status;
}

void ServoBusInterface::processVte(bool success) {
  /**
   *  This processes the data for voltage, temperature and error of the servos. It is mainly used as diagnostic message.
   */

  // prepare diagnostic msg
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_names_.size(); i++) {
    char level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string message = "OK";
    std::map<std::string, std::string> map;
    if (!success) {
      // the read of VT or error failed, we will publish this and not the values
      message = "No response";
      level = diagnostic_msgs::DiagnosticStatus::STALE;
      array.push_back(createServoDiagMsg(joint_ids_[i], level, message, map, joint_names_[i]));
      continue;
    }
    map.insert(std::make_pair("Input Voltage", std::to_string(current_input_voltage_[i])));
    if (current_input_voltage_[i] < warn_volt_) {
      message = "Power getting low";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Temperature", std::to_string(current_temperature_[i])));
    if (current_temperature_[i] > warn_temp_) {
      message = "Getting hot";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Error Byte", std::to_string(current_error_[i])));
    if (current_error_[i] != 0) {
      // some error is detected
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      message = "Error(s): ";
      // check which one. Values taken from dynamixel documentation
      char voltage_error = 0x1;
      char overheat_error = 0x4;
      char encoder_error = 0x8;
      char shock_error = 0x10;
      char overload_error = 0x20;
      if ((current_error_[i] & voltage_error) != 0) {
        message = message + "Voltage ";
      }
      if ((current_error_[i] & overheat_error) != 0) {
        message = message + "Overheat ";
      }
      if ((current_error_[i] & encoder_error) != 0) {
        message = message + "Encoder ";
      }
      if ((current_error_[i] & shock_error) != 0) {
        message = message + "Shock ";
      }
      if ((current_error_[i] & overload_error) != 0) {
        message = message + "Overload";
        // turn off torque on all motors
        // todo should also turn off power, but is not possible yet
        goal_torque_ = false;
        ROS_ERROR("OVERLOAD ERROR!!! OVERLOAD ERROR!!! OVERLOAD ERROR!!! In Motor %d", i);
        speakError(speak_pub_, "Overload Error!");
      }
    }

    array.push_back(createServoDiagMsg(joint_ids_[i], level, message, map, joint_names_[i]));
  }
  array_msg.status = array;
  diagnostic_pub_.publish(array_msg);
}

void ServoBusInterface::writeTorque(bool enabled) {
  /**
   * This writes the torque for all servos to the same value
   */
  //only set values if we're not in torqueless mode
  if (!torqueless_mode_) {
    std::vector<int32_t> torque(joint_names_.size(), enabled);
    int32_t *t = &torque[0];
    driver_->syncWrite("Torque_Enable", t);
    current_torque_ = enabled;
    goal_torque_ = enabled;
  }
}

void ServoBusInterface::writeTorqueForServos(std::vector<int32_t> torque) {
  /**
   * This writes the torque off all servos individually depended on the give vector.
   */

  //only set values if we're not in torqueless mode
  if (!torqueless_mode_) {
    // this actually writes each value in the torque vector
    // but the dynamixel_toolbox requires a reference to the first element
    int32_t *t = &torque[0];
    driver_->syncWrite("Torque_Enable", t);
  }
}

bool ServoBusInterface::syncReadPositions() {
  /**
   * Reads all position information with a single sync read
   */
  bool success;
  success = driver_->syncRead("Present_Position", data_sync_read_positions_);
  if (success) {
    for (int i = 0; i < joint_count_; i++) {
      // TODO test if this is required
      if (data_sync_read_positions_[i] == 0) {
        // a value of 0 is often a reading error, therefore we discard it
        // this should not cause issues when a motor is actually close to 0
        // since 1 bit only corresponds to + or - 0.1 deg
        continue;
      }
      double current_pos = driver_->convertValue2Radian(joint_ids_[i], data_sync_read_positions_[i]);
      if (current_pos < 3.15 && current_pos > -3.15) {
        //only write values which are possible
        current_position_[i] = current_pos;
      }
    }
  }
  return success;
}

bool ServoBusInterface::syncReadVelocities() {
  /**
   * Reads all velocity information with a single sync read
   */
  bool success;
  success = driver_->syncRead("Present_Velocity", data_sync_read_velocities_);
  if (success) {
    for (int i = 0; i < joint_count_; i++) {
      current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], data_sync_read_velocities_[i]);
    }
  }
  return success;
}

bool ServoBusInterface::syncReadEfforts() {
  /**
   * Reads all effort information with a single sync read
   */
  bool success;
  success = driver_->syncRead("Present_Current", data_sync_read_efforts_);
  if (success) {
    for (int i = 0; i < joint_count_; i++) {
      current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], data_sync_read_efforts_[i]);
    }
  }
  return success;
}

bool ServoBusInterface::syncReadPWMs() {
  /**
   * Reads all PWM information with a single sync read
   */
  bool success;
  success = driver_->syncRead("Present_PWM", data_sync_read_pwms_);
  if (success) {
    for (int i = 0; i < joint_count_; i++) {
      // the data is in int16
      // 100% is a value of 885
      // convert to range -1 to 1
      current_pwm_[i] = ((int16_t) data_sync_read_pwms_[i]) / 885.0;
    }
  }
  return success;
}

bool ServoBusInterface::syncReadError() {
  /**
   * Reads all error bytes with a single sync read
   */
  bool success;
  success = driver_->syncRead("Hardware_Error_Status", data_sync_read_error_);
  if (success) {
    for (int i = 0; i < joint_count_; i++) {
      current_error_[i] = data_sync_read_error_[i];
    }
  }
  return success;
}

bool ServoBusInterface::syncReadVoltageAndTemp() {
  /**
   * Reads all voltages and temperature information with a single sync read
   */
  std::vector<uint8_t> data;
  if (driver_->syncReadMultipleRegisters(144, 3, &data)) {
    uint16_t volt;
    uint8_t temp;
    for (int i = 0; i < joint_count_; i++) {
      volt = dxlMakeword(data[i * 3], data[i * 3 + 1]);
      temp = data[i * 3 + 2];
      // convert value to voltage
      current_input_voltage_[i] = volt * 0.1;
      // is already in °C
      current_temperature_[i] = temp;
    }
    return true;
  } else {
    return false;
  }
}

bool ServoBusInterface::syncReadAll() {
  /**
   * Reads all positions, velocities and efforts with a single sync read
   */
  if (driver_->syncReadMultipleRegisters(126, 10, &sync_read_all_data_)) {
    int16_t eff;
    uint32_t vel;
    uint32_t pos;
    for (int i = 0; i < joint_count_; i++) {
      eff = dxlMakeword(sync_read_all_data_[i * 10], sync_read_all_data_[i * 10 + 1]);
      vel = dxlMakedword(dxlMakeword(sync_read_all_data_[i * 10 + 2], sync_read_all_data_[i * 10 + 3]),
                         dxlMakeword(sync_read_all_data_[i * 10 + 4], sync_read_all_data_[i * 10 + 5]));
      pos = dxlMakedword(dxlMakeword(sync_read_all_data_[i * 10 + 6], sync_read_all_data_[i * 10 + 7]),
                         dxlMakeword(sync_read_all_data_[i * 10 + 8], sync_read_all_data_[i * 10 + 9]));
      current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], eff);
      current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], vel);
      double current_pos = driver_->convertValue2Radian(joint_ids_[i], pos);
      if (current_pos < 3.15 && current_pos > -3.15) {
        //only write values which are possible
        current_position_[i] = current_pos;
      }
    }
    return true;
  } else {
    return false;
  }
}

void ServoBusInterface::syncWritePosition() {
  /**
   * Writes all goal positions with a single sync write
   */
  float radian;
  for (size_t num = 0; num < joint_names_.size(); num++) {
    radian = goal_position_[num] - joint_mounting_offsets_[num] - joint_offsets_[num];
    sync_write_goal_position_[num] = driver_->convertRadian2Value(joint_ids_[num], radian);
  }
  driver_->syncWrite("Goal_Position", sync_write_goal_position_);
}

void ServoBusInterface::syncWriteVelocity() {
  /**
   * Writes all goal velocities with a single sync write
   */
  for (size_t num = 0; num < joint_names_.size(); num++) {
    sync_write_goal_velocity_[num] = driver_->convertVelocity2Value(joint_ids_[num], goal_velocity_[num]);
  }
  driver_->syncWrite("Goal_Velocity", sync_write_goal_velocity_);
}

void ServoBusInterface::syncWriteProfileVelocity() {
  /**
   * Writes all profile velocities with a single sync write
   */
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if (goal_velocity_[num] < 0) {
      // we want to set to maximum, which is 0
      sync_write_profile_velocity_[num] = 0;
    } else {
      // use max to prevent accidentially setting 0
      sync_write_profile_velocity_[num] = std::max(driver_->convertVelocity2Value(joint_ids_[num], goal_velocity_[num]), 1);
    }
  }
  driver_->syncWrite("Profile_Velocity", sync_write_profile_velocity_);
}

void ServoBusInterface::syncWriteProfileAcceleration() {
  /**
   * Writes all profile accelerations with a single sync write
   */
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if (goal_acceleration_[num] < 0) {
      // we want to set to maximum, which is 0
      sync_write_profile_acceleration_[num] = 0;
    } else {
      //572.9577952 for change of units, 214.577 rev/min^2 per LSB
      sync_write_profile_acceleration_[num] = std::max(static_cast<int>(goal_acceleration_[num] * 572.9577952 / 214.577), 1);
    }
  }
  driver_->syncWrite("Profile_Acceleration", sync_write_profile_acceleration_);
}

void ServoBusInterface::syncWriteCurrent() {
  /**
   * Writes all goal currents with a single sync write
   */
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if (goal_effort_[num] < 0) {
      // we want to set to maximum, which is different for MX-64 and MX-106
      if (driver_->getModelNum(joint_ids_[num]) == 311) {
        sync_write_goal_current_[num] = 1941;
      } else if (driver_->getModelNum(joint_ids_[num]) == 321) {
        sync_write_goal_current_[num] = 2047;
      } else {
        ROS_WARN("Maximal current for this dynamixel model is not defined");
      }
    } else {
      sync_write_goal_current_[num] = driver_->convertTorque2Value(joint_ids_[num], goal_effort_[num]);
    }
  }
  driver_->syncWrite("Goal_Current", sync_write_goal_current_);
}

void ServoBusInterface::syncWritePWM() {
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if (goal_effort_[num] < 0) {
      // we want to set to maximum
      sync_write_goal_pwm_[num] = 855;
    } else {
      sync_write_goal_pwm_[num] = goal_effort_[num] / 100.0 * 855.0;
    }
  }
  driver_->syncWrite("Goal_PWM", sync_write_goal_pwm_);
}
}
