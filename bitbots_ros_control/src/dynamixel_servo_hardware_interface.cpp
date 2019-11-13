#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control
{

DynamixelServoHardwareInterface::DynamixelServoHardwareInterface(){}

DynamixelServoHardwareInterface::DynamixelServoHardwareInterface(std::shared_ptr<DynamixelDriver>& driver)
  : first_cycle_(true), read_position_(true), read_velocity_(false), read_effort_(true){
  driver_ = driver;
}

bool DynamixelServoHardwareInterface::init(ros::NodeHandle& nh){
  /*
  * This initializes the hardware interface based on the values set in the config.
  * The servos are pinged to verify that a connection is present and to know which type of servo it is.
  */
  nh_ = nh;
  lost_servo_connection_ = false;
  read_vt_counter_ = 0;

  // Init subscriber / publisher
  switch_individual_torque_ = false;
  set_torque_sub_ = nh.subscribe<std_msgs::BoolConstPtr>("set_torque", 1, &DynamixelServoHardwareInterface::setTorqueCb, this, ros::TransportHints().tcpNoDelay());
  set_torque_indiv_sub_ = nh.subscribe<bitbots_msgs::JointTorque>("set_torque_individual", 1, &DynamixelServoHardwareInterface::individualTorqueCb, this, ros::TransportHints().tcpNoDelay());
  pwm_pub_ = nh.advertise<sensor_msgs::JointState>("/servo_PID_status", 10, true);
  diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  speak_pub_ = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1, true);

  torqueless_mode_ = nh.param("torqueless_mode", false);

  // Load dynamixel config from parameter server
  if (!loadDynamixels(nh)){
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

  // write ROM and RAM values if wanted
  if(nh.param("servos/set_ROM_RAM", false)){
    if (!writeROMRAM(nh)){
        ROS_WARN("Couldn't write ROM and RAM values to all servos.");
    }
    // magic sleep preventing problems after setting ROM values
    // not sure if still needed for never firmware, but better keep it to be save
    sleep(1);
  }

  // register ros_control interfaces
  for (unsigned int i = 0; i < joint_names_.size(); i++){
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &current_position_[i], &current_velocity_[i], &current_effort_[i]);
    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &goal_position_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &goal_velocity_[i]);
    jnt_vel_interface_.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &goal_effort_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);

    hardware_interface::PosVelAccCurJointHandle posvelacccur_handle(state_handle, &goal_position_[i], &goal_velocity_[i], &goal_acceleration_[i], &goal_effort_[i]);
    jnt_posvelacccur_interface_.registerHandle(posvelacccur_handle);

  }
  parent_->registerInterface(&jnt_state_interface_);
  if (control_mode_ == POSITION_CONTROL){
    // we use the posvelacccur interface to be compatible to the rest of our software
    // normally this should be a position interface
    parent_->registerInterface(&jnt_posvelacccur_interface_);
  } else if (control_mode_ == VELOCITY_CONTROL){
    parent_->registerInterface(&jnt_vel_interface_);
  } else if (control_mode_ == EFFORT_CONTROL){
    parent_->registerInterface(&jnt_eff_interface_);
  } else if(control_mode_ == CURRENT_BASED_POSITION_CONTROL ){
    parent_->registerInterface(&jnt_posvelacccur_interface_);
  }

  writeTorque(nh.param("servos/auto_torque", false));

  // init dynamic reconfigure
  dyn_reconf_server_ = new dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>(ros::NodeHandle("~/servos"));
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconfCallback, this, _1, _2);
  dyn_reconf_server_->setCallback(f);

  ROS_INFO("Hardware interface init finished.");
  return true;
}

bool DynamixelServoHardwareInterface::loadDynamixels(ros::NodeHandle& nh){
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
  ROS_INFO("Control mode: %s", control_mode.c_str() );
  if (!stringToControlMode(control_mode, control_mode_)) {
    ROS_ERROR_STREAM("Unknown control mode'" << control_mode << "'.");
    speakError(speak_pub_, "Wrong control mode specified");
    return false;
  }

  // get values to read
  nh.param("servos/read_position", read_position_, true);
  nh.param("servos/read_velocity", read_velocity_, false);
  nh.param("servos/read_effort", read_effort_, false);


  XmlRpc::XmlRpcValue dxls_xml;
  nh.getParam("servos/device_info", dxls_xml);
  ROS_ASSERT(dxls_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Convert dxls to native type: a vector of tuples with name and id for sorting purposes
  std::vector<std::pair<std::string, int>> dxls;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls_xml.begin(); it != dxls_xml.end(); ++it) {
    std::string name = it->first;
    XmlRpc::XmlRpcValue data = it->second;
    int id = data["id"];
    dxls.emplace_back(name, id);
  }

  // sort the servos by id. This way the servos will always be read and written in ID order later, making debug easier.
  std::sort(dxls.begin(), dxls.end(),
        [](std::pair<std::string, int>& a, std::pair<std::string, int>& b) { return a.second < b.second; });

  // iterate over all servos and load each into the driver
  for(std::pair<std::string, int> &joint : dxls) {
    std::string dxl_name = joint.first;
    joint_names_.push_back(dxl_name);
    ros::NodeHandle dxl_nh(nh, "servos/device_info/" + dxl_name);

    joint_mounting_offsets_.push_back(dxl_nh.param("mounting_offset", 0.0));
    joint_offsets_.push_back(dxl_nh.param("offset", 0.0));

    int motor_id = joint.second;

    int model_number;
    dxl_nh.getParam("model_number", model_number);
    uint16_t model_number_16 = uint16_t(model_number);
    uint16_t* model_number_16p = &model_number_16;

    std::map<std::string, std::string> map;
    map.insert(std::make_pair("Joint Name", dxl_name));

    //ping it to very that it's there and to add it to the driver
    if(!driver_->ping(uint8_t(motor_id), model_number_16p)){
      ROS_ERROR("Was not able to ping motor with id %d (%s)", motor_id, dxl_name.c_str());
      success = false;
      array.push_back(createServoDiagMsg(motor_id, diagnostic_msgs::DiagnosticStatus::STALE, "No ping response", map));
    }
    array.push_back(createServoDiagMsg(motor_id, diagnostic_msgs::DiagnosticStatus::OK, "Ping sucessful", map));
    joint_ids_.push_back(uint8_t(motor_id));
  }

  // generate a diagnostic message
  array_msg.status = array;
  diagnostic_pub_.publish(array_msg);

  return success;
}

bool DynamixelServoHardwareInterface::writeROMRAM(ros::NodeHandle& nh){
  /**
   * This method writes the ROM and RAM values specified in the config to all servos.
   */
  ROS_INFO("Writing ROM and RAM values");
  XmlRpc::XmlRpcValue dxls;
  nh.getParam("servos/ROM_RAM", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  bool sucess = true;
  int i = 0;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it){
    std::string register_name = (std::string)(it->first);
    int register_value;
    nh.getParam(("servos/ROM_RAM/" + register_name).c_str(), register_value);
    ROS_INFO("Setting %s on all servos to %d", register_name.c_str(), register_value);

    int* values = (int*)malloc(joint_names_.size() * sizeof(int));
    for (size_t num = 0; num < joint_names_.size(); num++) {
      values[num] = register_value;
    }
    driver_->addSyncWrite(register_name.c_str());
    sucess = sucess && driver_->syncWrite(register_name.c_str(), values);
    free(values);
  }
  return sucess;
}

diagnostic_msgs::DiagnosticStatus DynamixelServoHardwareInterface::createServoDiagMsg(int id, char level, std::string message, std::map<std::string, std::string> map){
  /**
   * Create a single Diagnostic message for one servo. This is used to build the array message which is then published.
   */
    diagnostic_msgs::DiagnosticStatus servo_status = diagnostic_msgs::DiagnosticStatus();
    servo_status.level = level;
    servo_status.name = "Servo " + std::to_string(id);
    servo_status.message = message;
    servo_status.hardware_id = std::to_string(100 + id);
    std::vector<diagnostic_msgs::KeyValue> keyValues = std::vector<diagnostic_msgs::KeyValue>();
    // itarate through map and save it into values
    for(auto const &ent1 : map) {
      diagnostic_msgs::KeyValue key_value = diagnostic_msgs::KeyValue();
      key_value.key = ent1.first;
      key_value.value = ent1.second;
      keyValues.push_back(key_value);
    }
    servo_status.values = keyValues;
    return servo_status;
}

void DynamixelServoHardwareInterface::processVte(bool success){
  /**
   *  This processes the data for voltage, temperature and error of the servos. It is mainly used as diagnostic message.
   */

  // prepare diagnostic msg
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_names_.size(); i++){
    char level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string message = "OK";
    std::map<std::string, std::string> map;
    map.insert(std::make_pair("Joint Name", joint_names_[i]));
    if(!success){
      // the read of VT or error failed, we will publish this and not the values
      message = "No response";
      level = diagnostic_msgs::DiagnosticStatus::STALE;
      array.push_back(createServoDiagMsg(joint_ids_[i], level, message, map));
      continue;
    }
    map.insert(std::make_pair("Input Voltage", std::to_string(current_input_voltage_[i])));
    if(current_input_voltage_[i] < warn_volt_){
      message = "Power getting low";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Temperature", std::to_string(current_temperature_[i])));
    if(current_temperature_[i] > warn_temp_){
      message = "Getting hot";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Error Byte", std::to_string(current_error_[i])));
    if(current_error_[i] != 0 ){
      // some error is detected
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      message = "Error(s): ";
      // check which one. Values taken from dynamixel documentation
      char voltage_error = 0x1;
      char overheat_error = 0x4;
      char encoder_error = 0x8;
      char shock_error = 0x10;
      char overload_error = 0x20;
      if((current_error_[i] & voltage_error) != 0){
        message = message + "Voltage ";
      }
      if((current_error_[i] & overheat_error) != 0){
        message = message + "Overheat ";
      }
      if((current_error_[i] & encoder_error) != 0){
        message = message + "Encoder ";
      }
      if((current_error_[i] & shock_error) != 0){
        message = message + "Shock ";
      }
      if((current_error_[i] & overload_error) != 0){
        message = message + "Overload";
        // turn off torque on all motors
        // todo should also turn off power, but is not possible yet
        goal_torque_ = false;
        ROS_ERROR("OVERLOAD ERROR!!! OVERLOAD ERROR!!! OVERLOAD ERROR!!! In Motor %d", i);
        speakError(speak_pub_, "Overload Error!");
      }
    }

    array.push_back(createServoDiagMsg(joint_ids_[i], level, message, map));
  }
  array_msg.status = array;
  diagnostic_pub_.publish(array_msg);
}

void DynamixelServoHardwareInterface::writeTorque(bool enabled){
  /**
   * This writes the torque for all servos to the same value
   */
  //only set values if we're not in torqueless mode
  if(!torqueless_mode_){
      std::vector<int32_t> torque(joint_names_.size(), enabled);
      int32_t* t = &torque[0];
      driver_->syncWrite("Torque_Enable", t);
      current_torque_ = enabled;
  }
}

void DynamixelServoHardwareInterface::writeTorqueForServos(std::vector<int32_t> torque){
  /**
   * This writes the torque off all servos individually depended on the give vector.
   */

  //only set values if we're not in torqueless mode
  if(!torqueless_mode_){
    // this actually writes each value in the torque vector
    // but the dynamixel_toolbox requires a reference to the first element
    int32_t* t = &torque[0];
    driver_->syncWrite("Torque_Enable", t);
  }
}


void DynamixelServoHardwareInterface::individualTorqueCb(bitbots_msgs::JointTorque msg){
  /**
   * Handles incomming JointTroque messages and remembers the requested torque configuration of the servos.
   * It will not directly written since this has to happen in the main write loop
   */
  if(torqueless_mode_){
    return;
  }

  // we save the goal torque value. It will be set during write process
  for(int i = 0; i < msg.joint_names.size(); i++){
    bool success = false;
    for(int j = 0; j < joint_names_.size(); j++){
      if(msg.joint_names[i] == joint_names_[j]){
        if(i < msg.joint_names.size()){
          goal_torque_individual_[j] = msg.on[i];
          success = true;
        }else{
          ROS_WARN("Somethings wrong with your message to set torques.");
        }
      }
    }
    if(!success){
      ROS_WARN("Couldn't set torque for servo %s ", msg.joint_names[i].c_str());
    }
  }
  switch_individual_torque_ = true;
}

void DynamixelServoHardwareInterface::setTorqueCb(std_msgs::BoolConstPtr enabled){
  /**
   * This saves the given required value, so that it can be written to the servos in the write method
   */
  goal_torque_ = enabled->data;
  for(int j = 0; j < joint_names_.size(); j++) {
    goal_torque_individual_[j] = enabled->data;
  }
}

bool DynamixelServoHardwareInterface::read(){
  /**
   * This is part of the main loop and handles reading of all connected devices
   */
  bool read_successful = true;

  // either read all values or a single one, depending on config
  if (read_position_ && read_velocity_ && read_effort_ ){
    if(syncReadAll()){
      for (size_t num = 0; num < joint_names_.size(); num++) {
        current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
      }
    } else{
      ROS_ERROR_THROTTLE(1.0, "Couldn't read all current joint values!");
      read_successful = false;
    }
  }else {
    if (read_position_) {
      if (syncReadPositions()) {
        for (size_t num = 0; num < joint_names_.size(); num++) {
          current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
        }
      } else{
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

    if (read_PWM_) {
      if (!syncReadPWMs()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current PWM!");
        driver_->reinitSyncReadHandler("Present_PWM");
      }else{
        sensor_msgs::JointState pwm_state = sensor_msgs::JointState();
        pwm_state.header.stamp = ros::Time::now();
        pwm_state.name = joint_names_;
        pwm_state.effort = current_pwm_;
        pwm_pub_.publish(pwm_state);
      }
    }
    }


  if (read_volt_temp_){
    if (read_vt_counter_ + 1 == vt_update_rate_){
      bool success = true;
      if(!syncReadVoltageAndTemp()){
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current input volatage and temperature!");
        success = false;
      }
      if(!syncReadError()){
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current error bytes!");
        success = false;
        driver_->reinitSyncReadHandler("Hardware_Error_Status");
      }
      processVte(success);
    }
    read_vt_counter_ = (read_vt_counter_ + 1) % vt_update_rate_;
  }

  if (first_cycle_){
    // when the servos have a goal position which is not the current position on startup
    // they will rapidly move to this position, possibly damaging the robot
    // therefore the goal position is set to the current position of the motors
    for(int i = 0; i < current_position_.size(); i++){
      goal_position_[i] = current_position_[i];
    }
    first_cycle_ = false;
  }

  // remember
  if (!read_successful) {
    lost_servo_connection_ = true;
    reading_errors_ += 1;
  }else{
    reading_successes_ += 1;
  }

  if(reading_errors_ + reading_successes_ > 200 &&
     (float) reading_errors_ / (float) (reading_successes_ + reading_errors_) > 0.05f){
    speakError(speak_pub_, "Multiple servo reading errors!");
    reading_errors_ = 0;
    reading_successes_ = 0;
  }

  if(reading_errors_ + reading_successes_ > 2000){
    reading_errors_ = 0;
    reading_successes_ = 0;
  }

  return read_successful;

}

void DynamixelServoHardwareInterface::write()
{
  /**
   * This is part of the mainloop and handles all the writing to the connected devices
   */

  //check if we have to switch the torque
  if(current_torque_ != goal_torque_){
    writeTorque(goal_torque_);
  }

  if(switch_individual_torque_){
    writeTorqueForServos(goal_torque_individual_);
    switch_individual_torque_ = false;
  }

  // reset torques if we lost connection to them
  if(lost_servo_connection_){
    ROS_INFO_THROTTLE(5, "resetting torque after lost connection");
    writeTorqueForServos(goal_torque_individual_);
    lost_servo_connection_ = false;
  }

  if (control_mode_ == POSITION_CONTROL){
    if(goal_effort_ != last_goal_effort_){
      syncWritePWM();
      last_goal_effort_ = goal_effort_;
    }

    if(goal_velocity_ != last_goal_velocity_){
      syncWriteProfileVelocity();
      last_goal_velocity_ = goal_velocity_;
    }

    if(goal_acceleration_ != last_goal_acceleration_){
      syncWriteProfileAcceleration();
      last_goal_acceleration_ = goal_acceleration_;
    }

    if(goal_position_!= last_goal_position_){
      syncWritePosition();
      last_goal_position_ =goal_position_;
    }
  } else if (control_mode_ == VELOCITY_CONTROL){
      syncWriteVelocity();
  } else if (control_mode_ == EFFORT_CONTROL){
      syncWriteCurrent();
  }else if (control_mode_ == CURRENT_BASED_POSITION_CONTROL){
    // only write things if it is necessary
    if(goal_effort_ != last_goal_effort_){
      syncWriteCurrent();
      last_goal_effort_ = goal_effort_;
    }

    if(goal_velocity_ != last_goal_velocity_){
      syncWriteProfileVelocity();
      last_goal_velocity_ = goal_velocity_;
    }

    if(goal_acceleration_ != last_goal_acceleration_){
      syncWriteProfileAcceleration();
      last_goal_acceleration_ = goal_acceleration_;
    }

    if(goal_position_!= last_goal_position_){
      syncWritePosition();
      last_goal_position_ =goal_position_;
    }

  }
}

void DynamixelServoHardwareInterface::setParent(hardware_interface::RobotHW* parent) {
  /**
   * We need the parent to be able to register the controller interface.
   */

  parent_ = parent;
}

bool DynamixelServoHardwareInterface::stringToControlMode(std::string _control_modestr, ControlMode& control_mode)
{
  /**
   * Helper method to parse strings to corresponding control modes
   */
  if (_control_modestr == "position"){
    control_mode = POSITION_CONTROL;
    return true;
  } else if (_control_modestr == "velocity"){
    control_mode = VELOCITY_CONTROL;
    return true;
  } else if (_control_modestr == "effort"){
    control_mode = EFFORT_CONTROL;
    return true;
  } else if (_control_modestr == "current_based"){
    control_mode = CURRENT_BASED_POSITION_CONTROL;
    return true;
  }  else {
    ROS_WARN("Trying to set unknown control mode");
    return false;
  }
}

void DynamixelServoHardwareInterface::switchDynamixelControlMode()
{
  /**
   * This method switches the control mode of all servos
   */

  // Torque on dynamixels has to be disabled to change operating mode
  // save last torque state for later
  bool torque_before_switch = current_torque_;
  writeTorque(false);
  // magic sleep to make sure that dynamixel have internally processed the request
  ros::Duration(0.5).sleep();

  int32_t value = 3;
  if (control_mode_ == POSITION_CONTROL){
    value = 3;;
  } else if (control_mode_ == VELOCITY_CONTROL){
    value = 1;
  } else if (control_mode_ == EFFORT_CONTROL){
    value = 0;
  }else if (control_mode_ == CURRENT_BASED_POSITION_CONTROL){
    value = 5;
  }else{
    ROS_WARN("control_mode is wrong, will use position control");
  }

  std::vector<int32_t> operating_mode(joint_names_.size(), value);
  int32_t* o = &operating_mode[0];
  driver_->syncWrite("Operating_Mode", o);

  ros::Duration(0.5).sleep();
  //reenable torque if it was previously enabled
  writeTorque(torque_before_switch);
}

bool DynamixelServoHardwareInterface::syncReadPositions(){
  /**
   * Reads all position information with a single sync read
   */
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Position", data);
  if(success){
    for(int i = 0; i < joint_count_; i++){
      // TODO test if this is required
      if (data[i] == 0){
        // a value of 0 is often a reading error, therefore we discard it
        // this should not cause issues when a motor is actually close to 0
        // since 1 bit only corresponds to + or - 0.1 deg
        continue;
      }
      double current_pos = driver_->convertValue2Radian(joint_ids_[i], data[i]);
      if(current_pos < 3.15 && current_pos > -3.15){
        //only write values which are possible
        current_position_[i] = current_pos;
      }
    }
  }
  free(data);
  return success;
}

bool DynamixelServoHardwareInterface::syncReadVelocities(){
  /**
   * Reads all velocity information with a single sync read
   */
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Velocity", data);
  if(success){
    for(int i = 0; i < joint_count_; i++){
      current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], data[i]);
    }
  }
  free(data);

  return success;
}

bool DynamixelServoHardwareInterface::syncReadEfforts() {
  /**
   * Reads all effort information with a single sync read
   */
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Current", data);
  if(success){
    for (int i = 0; i < joint_count_; i++) {
      current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], data[i]);
    }
  }
  free(data);

  return success;
}

bool DynamixelServoHardwareInterface::syncReadPWMs() {
    /**
     * Reads all PWM information with a single sync read
     */
    bool success;
    int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
    success = driver_->syncRead("Present_PWM", data);
    if(success){
      for (int i = 0; i < joint_count_; i++) {
        current_pwm_[i] = data[i];
      }
    }
    free(data);

    return success;
  }

bool DynamixelServoHardwareInterface::syncReadError(){
  /**
   * Reads all error bytes with a single sync read
   */
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Hardware_Error_Status", data);
  if(success){
    for (int i = 0; i < joint_count_; i++) {
      current_error_[i] = data[i];
    }
  }
  free(data);
  return success;
}

bool DynamixelServoHardwareInterface::syncReadVoltageAndTemp(){
  /**
   * Reads all voltages and temperature information with a single sync read
   */
  std::vector<uint8_t> data;
  if(driver_->syncReadMultipleRegisters(144, 3, &data)) {
    uint16_t volt;
    uint8_t temp;
    for (int i = 0; i < joint_count_; i++) {
      volt = dxlMakeword(data[i*3], data[i*3 + 1]);
      temp = data[i * 3 + 2];
      // convert value to voltage
      current_input_voltage_[i] = volt * 0.1;
      // is already in °C
      current_temperature_[i] = temp;
    }
    return true;
  }else{
    return false;
  }
}

bool DynamixelServoHardwareInterface::syncReadAll() {
  /**
   * Reads all positions, velocities and efforts with a single sync read
   */
  std::vector<uint8_t> data;
  if(driver_->syncReadMultipleRegisters(126, 10, &data)) {
    int16_t eff;
    uint32_t vel;
    uint32_t pos;
    for (int i = 0; i < joint_count_; i++) {
      eff = dxlMakeword(data[i*10], data[i*10 + 1]);
      vel = dxlMakedword(dxlMakeword(data[i*10 + 2], data[i*10 + 3]),
                         dxlMakeword(data[i*10 + 4], data[i*10 + 5]));
      pos = dxlMakedword(dxlMakeword(data[i*10 + 6], data[i*10 + 7]),
                         dxlMakeword(data[i*10 + 8], data[i*10 + 9]));
      current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], eff);
      current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], vel);
      double current_pos = driver_->convertValue2Radian(joint_ids_[i], pos);
      if(current_pos < 3.15 && current_pos > -3.15){
        //only write values which are possible
        current_position_[i] = current_pos;
      }
    }
    return true;
  }else{
    return false;
  }
}

void DynamixelServoHardwareInterface::syncWritePosition(){
  /**
   * Writes all goal positions with a single sync write
   */
  int* goal_position = (int*)malloc(joint_names_.size() * sizeof(int));
  float radian;
  for (size_t num = 0; num < joint_names_.size(); num++) {
    radian = goal_position_[num] - joint_mounting_offsets_[num] - joint_offsets_[num];
    goal_position[num] = driver_->convertRadian2Value(joint_ids_[num], radian);
  }
  driver_->syncWrite("Goal_Position", goal_position);
  free(goal_position);
}

void DynamixelServoHardwareInterface::syncWriteVelocity() {
  /**
   * Writes all goal velocities with a single sync write
   */
  int* goal_velocity = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    goal_velocity[num] = driver_->convertVelocity2Value(joint_ids_[num], goal_velocity_[num]);
  }
  driver_->syncWrite("Goal_Velocity", goal_velocity);
  free(goal_velocity);
}

void DynamixelServoHardwareInterface::syncWriteProfileVelocity() {
  /**
   * Writes all profile velocities with a single sync write
   */
  int* goal_velocity = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if(goal_velocity_[num] < 0){
      // we want to set to maximum, which is 0
      goal_velocity[num] = 0;
    }else{
      // use max to prevent accidentially setting 0
      goal_velocity[num] = std::max(driver_->convertVelocity2Value(joint_ids_[num], goal_velocity_[num]), 1);
    }
  }
  driver_->syncWrite("Profile_Velocity", goal_velocity);
  free(goal_velocity);
}

void DynamixelServoHardwareInterface::syncWriteProfileAcceleration() {
  /**
   * Writes all profile accelerations with a single sync write
   */
  int* goal_acceleration = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if(goal_acceleration_[num] < 0){
      // we want to set to maximum, which is 0
      goal_acceleration[num] = 0;
    }else{
      //572.9577952 for change of units, 214.577 rev/min^2 per LSB
      goal_acceleration[num] = std::max(static_cast<int>(goal_acceleration_[num] * 572.9577952 / 214.577), 1);
    }
  }
  driver_->syncWrite("Profile_Acceleration", goal_acceleration);
  free(goal_acceleration);
}

void DynamixelServoHardwareInterface::syncWriteCurrent() {
  /**
   * Writes all goal currents with a single sync write
   */
  int* goal_current = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if(goal_effort_[num] < 0){
      // we want to set to maximum, which is different for MX-64 and MX-106
      if(driver_->getModelNum(joint_ids_[num]) == 311){
        goal_current[num] = 1941;
      }else if (driver_->getModelNum(joint_ids_[num]) == 321){
        goal_current[num] = 2047;
      }else{
        ROS_WARN("Maximal current for this dynamixel model is not defined");
      }
    }else{
      goal_current[num] = driver_->convertTorque2Value(joint_ids_[num], goal_effort_[num]);
    }
  }
  driver_->syncWrite("Goal_Current", goal_current);
  free(goal_current);
}

void DynamixelServoHardwareInterface::syncWritePWM() {
  int* goal_current = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    if(goal_effort_[num] < 0){
      // we want to set to maximum
      goal_current[num] = 855;
    }else{
      goal_current[num] = goal_effort_[num] / 100.0 * 855.0;
    }
  }
  driver_->syncWrite("Goal_PWM", goal_current);
  free(goal_current);
}

void DynamixelServoHardwareInterface::reconfCallback(bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig &config, uint32_t level) {
  read_position_ = config.read_position;
  read_velocity_ = config.read_velocity;
  read_effort_ = config.read_effort;
  read_PWM_ = config.read_pwm;
  read_volt_temp_ = config.read_volt_temp;
  vt_update_rate_ = config.VT_update_rate;
  warn_temp_ = config.warn_temp;
  warn_volt_ = config.warn_volt;
}

}
