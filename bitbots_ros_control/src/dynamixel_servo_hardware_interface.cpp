#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))

namespace bitbots_ros_control
{

DynamixelServoHardwareInterface::DynamixelServoHardwareInterface()
  : first_cycle_(true), _read_position(true), _read_velocity(false), _read_effort(true){
}


void DynamixelServoHardwareInterface::set_driver(boost::shared_ptr<DynamixelDriver> driver){
  _driver = driver;
}


bool DynamixelServoHardwareInterface::init(ros::NodeHandle& nh){
  /*
  * This initializes the hardware interface based on the values set in the config.
  * The servos are pinged to verify that a connection is present and to know which type of servo it is.
  */
  _nh = nh;
  _lost_servo_connection = false;

  // Init subscriber / publisher
  _switch_individual_torque = false;
  _set_torque_sub = nh.subscribe<std_msgs::BoolConstPtr>("set_torque", 1, &DynamixelServoHardwareInterface::setTorque, this, ros::TransportHints().tcpNoDelay());
  _set_torque_indiv_sub = nh.subscribe<bitbots_msgs::JointTorque>("set_torque_individual", 1, &DynamixelServoHardwareInterface::individualTorqueCb, this, ros::TransportHints().tcpNoDelay());
  _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  _speak_pub = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1, this);

  _torquelessMode = nh.param("torquelessMode", false);

  // Load dynamixel config from parameter server
  if (!loadDynamixels(nh)){
    ROS_ERROR_STREAM("Failed to ping all motors.");
    speak("Failed to ping all motors.");
    return false;
  }  

  // init the different sync write and read commands that will maybe necessary
  _driver->addSyncWrite("Torque_Enable");
  _driver->addSyncWrite("Goal_Position");
  _driver->addSyncWrite("Goal_Velocity");
  _driver->addSyncWrite("Profile_Velocity");
  _driver->addSyncWrite("Profile_Acceleration");
  _driver->addSyncWrite("Goal_Current");
  _driver->addSyncWrite("Goal_PWM");
  _driver->addSyncWrite("Operating_Mode");
  _driver->addSyncRead("Present_Current");
  _driver->addSyncRead("Present_Velocity");
  _driver->addSyncRead("Present_Position");
  _driver->addSyncRead("Hardware_Error_Status");

  // Switch dynamixels to correct control mode (position, velocity, effort)
  switchDynamixelControlMode();

  // allocate correct memory for number of used joints
  _joint_count = _joint_names.size();
  _current_position.resize(_joint_count, 0);
  _current_velocity.resize(_joint_count, 0);
  _current_effort.resize(_joint_count, 0);
  _current_input_voltage.resize(_joint_count, 0);
  _current_temperature.resize(_joint_count, 0);
  _current_error.resize(_joint_count, 0);
  _goal_position.resize(_joint_count, 0);
  _goal_velocity.resize(_joint_count, 0);
  _goal_acceleration.resize(_joint_count, 0);
  _goal_effort.resize(_joint_count, 0);
  _goal_torque_individual.resize(_joint_count, 1);

  // write ROM and RAM values if wanted
  if(nh.param("dynamixels/set_ROM_RAM", false)){
    if (!writeROMRAM(nh)){
        ROS_WARN("Couldn't write ROM and RAM values to all servos.");
    }
    // magic sleep preventing problems after setting ROM values
    // not sure if still needed for never firmware, but better keep it to be save
    sleep(1);
  }

  // register ros_control interfaces
  for (unsigned int i = 0; i < _joint_names.size(); i++){
    hardware_interface::JointStateHandle state_handle(_joint_names[i], &_current_position[i], &_current_velocity[i], &_current_effort[i]);
    _jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &_goal_position[i]);
    _jnt_pos_interface.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &_goal_velocity[i]);
    _jnt_vel_interface.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &_goal_effort[i]);
    _jnt_eff_interface.registerHandle(eff_handle);

    hardware_interface::PosVelAccCurJointHandle posvelacccur_handle(state_handle, &_goal_position[i], &_goal_velocity[i], &_goal_acceleration[i], &_goal_effort[i]);
    _jnt_posvelacccur_interface.registerHandle(posvelacccur_handle);

  }
  registerInterface(&_jnt_state_interface);
  if (_control_mode == PositionControl){
    //registerInterface(&_jnt_pos_interface);
    //todo hack
    registerInterface(&_jnt_posvelacccur_interface);
  } else if (_control_mode == VelocityControl){
    registerInterface(&_jnt_vel_interface);
  } else if (_control_mode == EffortControl){
    registerInterface(&_jnt_eff_interface);
  } else if(_control_mode == CurrentBasedPositionControl ){
    registerInterface(&_jnt_posvelacccur_interface);
  }

  writeTorque(nh.param("dynamixels/auto_torque", false));

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
  nh.getParam("dynamixels/control_mode", control_mode);
  ROS_INFO("Control mode: %s", control_mode.c_str() );
  if (!stringToControlMode(control_mode, _control_mode)) {
    ROS_ERROR_STREAM("Unknown control mode'" << control_mode << "'.");
    speak("Wrong control mode specified");
    return false;
  }

  // get values to read
  nh.param("read_position", _read_position, true);
  nh.param("read_velocity", _read_velocity, false);
  nh.param("read_effort", _read_effort, false);


  XmlRpc::XmlRpcValue dxls_xml;
  nh.getParam("dynamixels/device_info", dxls_xml);
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
    _joint_names.push_back(dxl_name);
    ros::NodeHandle dxl_nh(nh, "dynamixels/device_info/" + dxl_name);

    _joint_mounting_offsets.push_back(dxl_nh.param("mounting_offset", 0.0));
    _joint_offsets.push_back(dxl_nh.param("offset", 0.0));

    int motor_id = joint.second;

    int model_number;
    dxl_nh.getParam("model_number", model_number);
    uint16_t model_number_16 = uint16_t(model_number);
    uint16_t* model_number_16p = &model_number_16;
    
    std::map<std::string, std::string> map;
    map.insert(std::make_pair("Joint Name", dxl_name));

    //ping it to very that it's there and to add it to the driver
    if(!_driver->ping(uint8_t(motor_id), model_number_16p)){
      ROS_ERROR("Was not able to ping motor with id %d (%s)", motor_id, dxl_name.c_str());
      success = false;
      array.push_back(createServoDiagMsg(motor_id, diagnostic_msgs::DiagnosticStatus::STALE, "No ping response", map));
    }
    array.push_back(createServoDiagMsg(motor_id, diagnostic_msgs::DiagnosticStatus::OK, "Ping sucessful", map));
    _joint_ids.push_back(uint8_t(motor_id));
  }

  // generate a diagnostic message
  array_msg.status = array;
  _diagnostic_pub.publish(array_msg);

  return success;
}

bool DynamixelServoHardwareInterface::writeROMRAM(ros::NodeHandle& nh){
  /**
   * This method writes the ROM and RAM values specified in the config to all servos.
   */
  ROS_INFO("Writing ROM and RAM values");
  XmlRpc::XmlRpcValue dxls;
  nh.getParam("dynamixels/ROM_RAM", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  bool sucess = true;
  int i = 0;  
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it){
    std::string register_name = (std::string)(it->first);
    int register_value;
    nh.getParam(("dynamixels/ROM_RAM/" + register_name).c_str(), register_value);
    ROS_INFO("Setting %s on all servos to %d", register_name.c_str(), register_value);

    int* values = (int*)malloc(_joint_names.size() * sizeof(int));
    for (size_t num = 0; num < _joint_names.size(); num++) {
      values[num] = register_value;      
    }
    _driver->addSyncWrite(register_name.c_str());
    sucess = sucess && _driver->syncWrite(register_name.c_str(), values);
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
    servo_status.hardware_id = "" + std::to_string(100 + id);
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

void DynamixelServoHardwareInterface::processVTE(bool success){
  /**
   *  This processes the data for voltage, temperature and error of the servos. It is mainly used as diagnostic message.
   */

  // prepare diagnostic msg
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < _joint_names.size(); i++){
    char level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string message = "OK";
    std::map<std::string, std::string> map;
    map.insert(std::make_pair("Joint Name", _joint_names[i]));
    if(!success){
      // the read of VT or error failed, we will publish this and not the values
      message = "No response";
      level = diagnostic_msgs::DiagnosticStatus::STALE;
      array.push_back(createServoDiagMsg(_joint_ids[i], level, message, map));
      continue;
    }
    map.insert(std::make_pair("Input Voltage", std::to_string(_current_input_voltage[i])));
    if(_current_input_voltage[i] < _warn_volt){
      message = "Power getting low";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Temperature", std::to_string(_current_temperature[i])));    
    if(_current_temperature[i] > _warn_temp){
      message = "Getting hot";
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    map.insert(std::make_pair("Error Byte", std::to_string(_current_error[i])));
    if(_current_error[i] != 0 ){
      // some error is detected
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      message = "Error(s): ";
      // check which one. Values taken from dynamixel documentation
      char voltage_error = 0x1;
      char overheat_error = 0x4;
      char encoder_error = 0x8;
      char shock_error = 0x10;
      char overload_error = 0x20;
      if((_current_error[i] & voltage_error) != 0){
        message = message + "Voltage ";        
      }
      if((_current_error[i] & overheat_error) != 0){
        message = message + "Overheat ";        
      }
      if((_current_error[i] & encoder_error) != 0){
        message = message + "Encoder ";        
      }
      if((_current_error[i] & shock_error) != 0){
        message = message + "Shock ";        
      }
      if((_current_error[i] & overload_error) != 0){
        message = message + "Overload";
        // turn off torque on all motors
        // todo should also turn off power, but is not possible yet
        goal_torque_ = false;
        ROS_ERROR("OVERLOAD ERROR!!! OVERLOAD ERROR!!! OVERLOAD ERROR!!! In Motor %d", i);
        speak("Overload Error!");
      }
    }
    
    array.push_back(createServoDiagMsg(_joint_ids[i], level, message, map));
  }
  array_msg.status = array;
  _diagnostic_pub.publish(array_msg);
}

void DynamixelServoHardwareInterface::writeTorque(bool enabled){
  /**
   * This writes the torque for all servos to the same value
   */
  //only set values if we're not in torqueless mode
  if(!_torquelessMode){
      std::vector<int32_t> torque(_joint_names.size(), enabled);
      int32_t* t = &torque[0];
      _driver->syncWrite("Torque_Enable", t);
      current_torque_ = enabled;
  }
}

void DynamixelServoHardwareInterface::writeTorqueForServos(std::vector<int32_t> torque){
  /**
   * This writes the torque off all servos individually depended on the give vector.
   */
  //only set values if we're not in torqueless mode
  if(!_torquelessMode){
    int32_t* t = &torque[0];
    _driver->syncWrite("Torque_Enable", t);
  }
}


void DynamixelServoHardwareInterface::individualTorqueCb(bitbots_msgs::JointTorque msg){
  /**
   * Handles incomming JointTroque messages and remembers the requested torque configuration of the servos.
   * It will not directly written since this has to happen in the main write loop
   */
  if(_torquelessMode){
    return;
  }

  // we save the goal torque value. It will be set during write process
  for(int i = 0; i < msg.joint_names.size(); i++){
    bool success = false;
    for(int j = 0; j < _joint_names.size(); j++){
      if(msg.joint_names[i] == _joint_names[j]){
        if(i < msg.joint_names.size()){
          _goal_torque_individual[j] = msg.on[i];
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
  _switch_individual_torque = true;
}

void DynamixelServoHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled){
  /**
   * This saves the given required value, so that it can be written to the servos in the write method
   */
  goal_torque_ = enabled->data;
}

bool DynamixelServoHardwareInterface::read(){
  /**
   * This is part of the main loop and handles reading of all connected devices
   */
  bool read_successful = true;

  // either read all values or a single one, depending on config
  if (_read_position && _read_velocity && _read_effort ){
    if(syncReadAll()){
      for (size_t num = 0; num < _joint_names.size(); num++) {
          _current_position[num] += _joint_mounting_offsets[num] + _joint_offsets[num];
      }
    } else{
      ROS_ERROR_THROTTLE(1.0, "Couldn't read all current joint values!");
      read_successful = false;
    }
  }else {
    if (_read_position) {
      if (syncReadPositions()) {
        for (size_t num = 0; num < _joint_names.size(); num++) {
            _current_position[num] += _joint_mounting_offsets[num] + _joint_offsets[num];
        }
      } else{
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint position!");
        _driver->reinitSyncReadHandler("Present_Position");
        read_successful = false;
      }
    }

    if (_read_velocity) {
      if (!syncReadVelocities()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint velocity!");
        _driver->reinitSyncReadHandler("Present_Velocity");
        read_successful = false;
      }
    }

    if (_read_effort) {
      if (!syncReadEfforts()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint effort!");
        _driver->reinitSyncReadHandler("Present_Current");
        read_successful = false;
      }
    }
  }

  if (_read_volt_temp){
    if (_read_VT_counter > _VT_update_rate){
      bool success = true;
      if(!syncReadVoltageAndTemp()){
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current input volatage and temperature!");
        success = false;
      }
      if(!syncReadError()){
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current error bytes!");  
        success = false;
        _driver->reinitSyncReadHandler("Hardware_Error_Status");
      }
      processVTE(success);
      _read_VT_counter = 0;
    }else{
      _read_VT_counter++;
    }
  }

  if (first_cycle_){
    // prevent jerky motions on startup
    _goal_position = _current_position;
    first_cycle_ = false;
  }

  // remember 
  if (!read_successful) {
    _lost_servo_connection = true;
    _reading_errors += 1;
  }else{
    _reading_successes += 1;
  }

  if(_reading_errors + _reading_successes > 200 && _reading_errors / _reading_successes > 0.05){
    speak("Multiple servo reading errors!");
    _reading_errors = 0;
    _reading_successes = 0;
  }

  if(_reading_errors + _reading_successes > 2000){
    _reading_errors = 0;
    _reading_successes = 0;
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

  if(_switch_individual_torque){
    writeTorqueForServos(_goal_torque_individual);
    _switch_individual_torque = false;
  }

  // reset torques if we lost connection to them
  if(_lost_servo_connection){
    ROS_INFO_THROTTLE(5, "resetting torque after lost connection");
    writeTorqueForServos(_goal_torque_individual);
    _lost_servo_connection = false;
  }

  if (_control_mode == PositionControl){
    if(_goal_effort != _last_goal_effort){
      syncWritePWM();
      _last_goal_effort = _goal_effort;
    }

    if(_goal_velocity != _last_goal_velocity){
      syncWriteProfileVelocity();
      _last_goal_velocity = _goal_velocity;
    }
    
    if(_goal_acceleration != _last_goal_acceleration){
      syncWriteProfileAcceleration();
      _last_goal_acceleration = _goal_acceleration;
    }

    if(_goal_position!= _last_goal_position){
      syncWritePosition();
      _last_goal_position =_goal_position;
    }
  } else if (_control_mode == VelocityControl){
      syncWriteVelocity();
  } else if (_control_mode == EffortControl){
      syncWriteCurrent();
  }else if (_control_mode == CurrentBasedPositionControl){
    // only write things if it is necessary
    if(_goal_effort != _last_goal_effort){
      syncWriteCurrent();
      _last_goal_effort = _goal_effort;
    }

    if(_goal_velocity != _last_goal_velocity){
      syncWriteProfileVelocity();
      _last_goal_velocity = _goal_velocity;
    }
    
    if(_goal_acceleration != _last_goal_acceleration){
      syncWriteProfileAcceleration();
      _last_goal_acceleration = _goal_acceleration;
    }

    if(_goal_position!= _last_goal_position){
      syncWritePosition();
      _last_goal_position =_goal_position;
    }

  }
}

void DynamixelServoHardwareInterface::speak(std::string text){
  /**
   *  Helper method to send a message for text-to-speech output
   */
  humanoid_league_msgs::Speak msg = humanoid_league_msgs::Speak();
  msg.text = text;
  msg.priority = humanoid_league_msgs::Speak::HIGH_PRIORITY;
  _speak_pub.publish(msg);
}

bool DynamixelServoHardwareInterface::stringToControlMode(std::string _control_modestr, ControlMode& control_mode)
{
  /**
   * Helper method to parse strings to corresponding control modes
   */
  if (_control_modestr == "position"){
    control_mode = PositionControl;
    return true;
  } else if (_control_modestr == "velocity"){
    control_mode = VelocityControl;
    return true;
  } else if (_control_modestr == "effort"){
    control_mode = EffortControl;
    return true;
  } else if (_control_modestr == "current_based"){
    control_mode = CurrentBasedPositionControl;
    return true;
  }  else {
    ROS_WARN("Trying to set unknown control mode");
    return false;
  }
}

bool DynamixelServoHardwareInterface::switchDynamixelControlMode()
{
  /**
   * This method switches the control mode of all servos
   */
  if(_onlySensors){
    return true;
  }

  // Torque on dynamixels has to be disabled to change operating mode
  // save last torque state for later
  bool torque_before_switch = current_torque_;
  writeTorque(false);
  // magic sleep to make sure that dynamixel have internally processed the request
  ros::Duration(0.5).sleep();

  int32_t value = 3;
  if (_control_mode == PositionControl){
    value = 3;;
  } else if (_control_mode == VelocityControl){
    value = 1;
  } else if (_control_mode == EffortControl){
    value = 0;
  }else if (_control_mode == CurrentBasedPositionControl){
    value = 5;
  }else{
    ROS_WARN("control_mode is wrong, will use position control");
  }

  std::vector<int32_t> operating_mode(_joint_names.size(), value);
  int32_t* o = &operating_mode[0];
  _driver->syncWrite("Operating_Mode", o);

  ros::Duration(0.5).sleep();
  //reenable torque if it was previously enabled
  writeTorque(torque_before_switch);
}

bool DynamixelServoHardwareInterface::syncReadPositions(){
  /**
   * Reads all position information with a single sync read
   */
  bool success;
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Position", data);
  for(int i = 0; i < _joint_count; i++){
    if (data[i] == 0){
      // this is most propably an reading error
      // TODO better solution for this hack
      continue;
    }
    double current_pos = _driver->convertValue2Radian(_joint_ids[i], data[i]);
    if(current_pos < 3.15 && current_pos > -3.15){
      //only write values which are possible
      _current_position[i] = current_pos;
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
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Velocity", data);
  for(int i = 0; i < _joint_count; i++){
    _current_velocity[i] = _driver->convertValue2Velocity(_joint_ids[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelServoHardwareInterface::syncReadEfforts() {
  /**
   * Reads all effort information with a single sync read
   */
  bool success; //todo maybe 16bit has to be used in stead, like in the readAll method
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Current", data);
  for (int i = 0; i < _joint_count; i++) {
    _current_effort[i] = _driver->convertValue2Torque(_joint_ids[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelServoHardwareInterface::syncReadError(){
  /**
   * Reads all error bytes with a single sync read
   */
  bool success; 
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Hardware_Error_Status", data);
  for (int i = 0; i < _joint_count; i++) {
    _current_error[i] = data[i];
  }
  free(data);
  return success;
}

bool DynamixelServoHardwareInterface::syncReadVoltageAndTemp(){
  /**
   * Reads all voltages and temperature information with a single sync read
   */
  std::vector<uint8_t> data;
  if(_driver->syncReadMultipleRegisters(144, 3, &data)) {
    uint16_t volt;
    uint8_t temp;    
    for (int i = 0; i < _joint_count; i++) {
      volt = DXL_MAKEWORD(data[i * 3], data[i * 3 + 1]);
      temp = data[i * 3 + 2];
      // convert value to voltage
      _current_input_voltage[i] = volt * 0.1;
      // is already in °C
      _current_temperature[i] = temp;
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
  if(_driver->syncReadMultipleRegisters(126, 10, &data)) {
    int16_t eff;
    uint32_t vel;
    uint32_t pos;
    for (int i = 0; i < _joint_count; i++) {
      eff = DXL_MAKEWORD(data[i * 10], data[i * 10 + 1]);
      vel = DXL_MAKEDWORD(DXL_MAKEWORD(data[i * 10 + 2], data[i * 10 + 3]),
                          DXL_MAKEWORD(data[i * 10 + 4], data[i * 10 + 5]));
      pos = DXL_MAKEDWORD(DXL_MAKEWORD(data[i * 10 + 6], data[i * 10 + 7]),
                          DXL_MAKEWORD(data[i * 10 + 8], data[i * 10 + 9]));
      _current_effort[i] = _driver->convertValue2Torque(_joint_ids[i], eff);
      _current_velocity[i] = _driver->convertValue2Velocity(_joint_ids[i], vel);
      double current_pos = _driver->convertValue2Radian(_joint_ids[i], pos);
      if(current_pos < 3.15 && current_pos > -3.15){
        //only write values which are possible
        _current_position[i] = current_pos;
      }
    }
    return true;
  }else{
    return false;
  }
}

bool DynamixelServoHardwareInterface::syncWritePosition(){
  /**
   * Writes all goal positions with a single sync write
   */
  int* goal_position = (int*)malloc(_joint_names.size() * sizeof(int));
  float radian;
  for (size_t num = 0; num < _joint_names.size(); num++) {
    radian = _goal_position[num] - _joint_mounting_offsets[num] - _joint_offsets[num];
    goal_position[num] = _driver->convertRadian2Value(_joint_ids[num], radian);
  }
  _driver->syncWrite("Goal_Position", goal_position);
  free(goal_position);
}

bool DynamixelServoHardwareInterface::syncWriteVelocity() {
  /**
   * Writes all goal velocities with a single sync write
   */
  int* goal_velocity = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    goal_velocity[num] = _driver->convertVelocity2Value(_joint_ids[num], _goal_velocity[num]);
  }
  _driver->syncWrite("Goal_Velocity", goal_velocity);
  free(goal_velocity);
}

bool DynamixelServoHardwareInterface::syncWriteProfileVelocity() {
  /**
   * Writes all profile velocities with a single sync write
   */
  int* goal_velocity = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    if(_goal_velocity[num] < 0){
      // we want to set to maximum, which is 0
      goal_velocity[num] = 0;  
    }else{
      // use max to prevent accidentially setting 0
      goal_velocity[num] = std::max(_driver->convertVelocity2Value(_joint_ids[num], _goal_velocity[num]), 1);      
    }
  }
  _driver->syncWrite("Profile_Velocity", goal_velocity);
  free(goal_velocity);
}

bool DynamixelServoHardwareInterface::syncWriteProfileAcceleration() {
  /**
   * Writes all profile accelerations with a single sync write
   */
  int* goal_acceleration = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    if(_goal_acceleration[num] < 0){
      // we want to set to maximum, which is 0
      goal_acceleration[num] = 0;  
    }else{
      //572.9577952 for change of units, 214.577 rev/min^2 per LSB
      goal_acceleration[num] = std::max(static_cast<int>(_goal_acceleration[num] * 572.9577952 / 214.577), 1);
    }
  }
  _driver->syncWrite("Profile_Acceleration", goal_acceleration);
  free(goal_acceleration);
}

bool DynamixelServoHardwareInterface::syncWriteCurrent() {
  /**
   * Writes all goal currents with a single sync write
   */
  int* goal_current = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    if(_goal_effort[num] < 0){
      // we want to set to maximum, which is different for MX-64 and MX-106      
      if(_driver->getModelNum(_joint_ids[num]) == 311){
        goal_current[num] = 1941;
      }else if (_driver->getModelNum(_joint_ids[num]) == 321){
        goal_current[num] = 2047;
      }else{
        ROS_WARN("Maximal current for this dynamixel model is not defined");
      }      
    }else{
      goal_current[num] = _driver->convertTorque2Value(_joint_ids[num], _goal_effort[num]); 
    }    
  }
  _driver->syncWrite("Goal_Current", goal_current);
  free(goal_current);
}

bool DynamixelServoHardwareInterface::syncWritePWM() {
  int* goal_current = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    if(_goal_effort[num] < 0){
      // we want to set to maximum    
      goal_current[num] = 855;
    }else{
      goal_current[num] = _goal_effort[num] / 100.0 * 855.0; 
    }    
  }
  _driver->syncWrite("Goal_PWM", goal_current);
  free(goal_current);
}


/*bool DynamixelHardwareInterface::syncWriteAll(){
  int goal_position;
  int goal_velocity;
  int goal_acceleration;
  int goal_current;
  float radian;
  std::vector<uint8_t*> data;

  for (size_t num = 0; num < _joint_names.size(); num++) {
    unit8_t * d;
    radian = _goal_position[num] - _joint_mounting_offsets[num] - _joint_offsets[num];
    goal_position = _driver->convertRadian2Value(_joint_ids[num], radian);
    goal_velocity = _driver->convertRadian2Value(_joint_ids[num], _goal_velocity[num]);
    goal_acceleration = _goal_acceleration[num] / 60 / 60 / (2*M_PI) / 214.577; //214.577 rev/min^2 per LSB
    goal_current = _driver->convertRadian2Value(_joint_ids[num], _goal_effort[num]);
    //todo fill data
  }
  _driver->syncReadMultipleRegisters(102, 18, data);

}*/

void DynamixelServoHardwareInterface::reconf_callback(bitbots_ros_control::bitbots_ros_control_paramsConfig &config, uint32_t level) {
  _read_position = config.read_position;
  _read_velocity = config.read_velocity;
  _read_effort = config.read_effort;
  _read_imu = config.read_imu;
  _read_volt_temp = config.read_volt_temp;
  _read_pressure = config.read_pressure;
  _VT_update_rate = config.VT_update_rate;
  _warn_temp = config.warn_temp;
  _warn_volt = config.warn_volt;  
}

}
