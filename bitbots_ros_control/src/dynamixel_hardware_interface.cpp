#include <bitbots_ros_control/dynamixel_hardware_interface.h>
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))

#define gravity 9.80665

namespace bitbots_ros_control
{

DynamixelHardwareInterface::DynamixelHardwareInterface()
  : first_cycle_(true), _read_position(true), _read_velocity(false), _read_effort(true), _driver(new DynamixelDriver())
{}

bool DynamixelHardwareInterface::init(ros::NodeHandle& nh)
{

  // Init subscriber / publisher
  _set_torque_sub = nh.subscribe<std_msgs::BoolConstPtr>("set_torque", 1, &DynamixelHardwareInterface::setTorque, this);
  _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, this);
  _status_board.name = "DXL_board";
  _status_board.hardware_id = 1;
  _status_IMU.name = "IMU";
  _status_IMU.hardware_id = 2;

  // init driver
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  std::string port_name;
  nh.getParam("dynamixels/port_info/port_name", port_name);
  int baudrate;
  nh.getParam("dynamixels/port_info/baudrate", baudrate);
  _driver->init(port_name.c_str(), uint32_t(baudrate));
  float protocol_version;
  nh.getParam("dynamixels/port_info/protocol_version", protocol_version);
  
  _driver->setPacketHandler(protocol_version);
  
  // alloc memory for imu values
  _orientation = (double*) malloc(4 * sizeof(double));
  std::fill(_orientation, _orientation+4, 0);
  _orientation_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_orientation_covariance, _orientation_covariance+9, 0);
  _angular_velocity = (double*) malloc(3 * sizeof(double));
  std::fill(_angular_velocity, _angular_velocity+3, 0);
  _angular_velocity_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_angular_velocity_covariance, _angular_velocity_covariance+9, 0);
  _linear_acceleration = (double*) malloc(3 * sizeof(double));
  std::fill(_linear_acceleration, _linear_acceleration+3, 0);
  _linear_acceleration_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_linear_acceleration_covariance, _linear_acceleration_covariance+9, 0);
  ROS_INFO("3");

  std::string imu_name;
  std::string imu_frame;
  nh.getParam("IMU/name", imu_name);
  nh.getParam("IMU/frame", imu_frame);
  nh.getParam("read_imu", _read_imu);
  hardware_interface::ImuSensorHandle imu_handle(imu_name, imu_frame, _orientation, _orientation_covariance, _angular_velocity, _angular_velocity_covariance, _linear_acceleration, _linear_acceleration_covariance);
  _imu_interface.registerHandle(imu_handle);
  registerInterface(&_imu_interface);

  _onlyIMU = nh.param("onlyIMU", false);
  if(_onlyIMU){
    ROS_WARN("ignoring servo errors since only IMU is set to true!");
    return true;
  }

  // Load dynamixel config from parameter server
  bool onlyImu = false;
  nh.getParam("IMU/onlyImu", onlyImu);
  if(!onlyImu)
  {
    if (!loadDynamixels(nh))
    {
      ROS_ERROR_STREAM("Failed to ping all motors.");
      return false;
    }
  }

  // Switch dynamixels to correct control mode (position, velocity, effort)
  switchDynamixelControlMode();

  _joint_count = _joint_names.size();
  _current_position.resize(_joint_count, 0);
  _current_velocity.resize(_joint_count, 0);
  _current_effort.resize(_joint_count, 0);
  _goal_position.resize(_joint_count, 0);
  _goal_velocity.resize(_joint_count, 0);
  _goal_effort.resize(_joint_count, 0);
  // register interfaces
  for (unsigned int i = 0; i < _joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle state_handle(_joint_names[i], &_current_position[i], &_current_velocity[i], &_current_effort[i]);
    _jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &_goal_position[i]);
    _jnt_pos_interface.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &_goal_velocity[i]);
    _jnt_vel_interface.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &_goal_effort[i]);
    _jnt_eff_interface.registerHandle(eff_handle);

  }
  registerInterface(&_jnt_state_interface);
  if (_control_mode == PositionControl)
  {
    registerInterface(&_jnt_pos_interface);
  } else if (_control_mode == VelocityControl)
  {
    registerInterface(&_jnt_vel_interface);
  } else if (_control_mode == EffortControl)
  {
    registerInterface(&_jnt_eff_interface);
  }
  setTorque(nh.param("dynamixels/auto_torque", false));

  _driver->addSyncWrite("Torque_Enable");
  _driver->addSyncWrite("Goal_Position");
  _driver->addSyncWrite("Goal_Velocity");
  _driver->addSyncWrite("Goal_Current");
  _driver->addSyncWrite("Operating_Mode");
  _driver->addSyncRead("Present_Current");
  _driver->addSyncRead("Present_Velocity");
  _driver->addSyncRead("Present_Position");

  ROS_INFO("Hardware interface init finished.");
  return true;
}

bool DynamixelHardwareInterface::loadDynamixels(ros::NodeHandle& nh)
{
  /*
  Load config and try to ping servos to test if everything is correct.
  Adds all dynamixel to the driver if they are pingable.
  */
  bool success = true;


  // prepare diagnostic msg
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();

  // get control mode
  std::string control_mode;
  nh.getParam("dynamixels/control_mode", control_mode);
  if (!stringToControlMode(control_mode, _control_mode)) {
    ROS_ERROR_STREAM("Unknown control mode'" << control_mode << "'.");
    return false;
  }

  // get values to read
  nh.param("read_position", _read_position, true);
  nh.param("read_velocity", _read_velocity, false);
  nh.param("read_effort", _read_effort, false);


  XmlRpc::XmlRpcValue dxls;
  nh.getParam("dynamixels/device_info", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  int i = 0;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it)
  {

    std::string dxl_name = (std::string)(it->first);
    _joint_names.push_back(dxl_name);
    ros::NodeHandle dxl_nh(nh, "dynamixels/device_info/" + dxl_name);

    _joint_mounting_offsets.push_back(dxl_nh.param("mounting_offset", 0.0));
    _joint_offsets.push_back(dxl_nh.param("offset", 0.0));

    int motor_id;
    dxl_nh.getParam("id", motor_id);

    int model_number;
    dxl_nh.getParam("model_number", model_number);
    uint16_t model_number_16 = uint16_t(model_number);
    uint16_t* model_number_16p = &model_number_16;

    //ping it to very that it's there and to add it to the driver
    if(!_driver->ping(uint8_t(motor_id), model_number_16p)){
      ROS_ERROR("Was not able to ping motor with id %d", motor_id);
      success = false;
    }
    _joint_ids.push_back(uint8_t(motor_id));
    i++;
  }

  if(!success){
    return false;
  }
  _status_board.level = diagnostic_msgs::DiagnosticStatus::OK;
  _status_board.message = "DXL board started";
  array.push_back(_status_board);
  array_msg.status = array;

  return success;
}

diagnostic_msgs::DiagnosticStatus createServoDiagMsg(int id, char level, char* message, std::map<std::string, std::string> map){
    diagnostic_msgs::DiagnosticStatus servo_status = diagnostic_msgs::DiagnosticStatus();
    servo_status.level = level;
    servo_status.name = ("Servo %f", id);
    servo_status.message = message;
    servo_status.hardware_id = 100 + id;
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

void DynamixelHardwareInterface::setTorque(bool enabled)
{
  std::vector<int32_t> torque(_joint_names.size(), enabled);
  int32_t* t = &torque[0];
  _driver->syncWrite("Torque_Enable", t);
  current_torque_ = enabled;
}

void DynamixelHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled)
{
  // we save the goal torque value. It will be set during write process
  goal_torque_ = enabled->data;
}

void DynamixelHardwareInterface::read()
{
  if(_read_imu){
      if(!readImu()){
          ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
      }
  }
  if(_onlyIMU){
    return;
  }
  // either read all values or a single one, depending on config
  if (_read_position && _read_velocity && _read_effort ){
    if(syncReadAll()){
      for (size_t num = 0; num < _joint_names.size(); num++)
        _current_position[num] += _joint_mounting_offsets[num] + _joint_offsets[num];
    } else
      ROS_ERROR_THROTTLE(1.0, "Couldn't read all current joint values!");
  }else {
    if (_read_position) {
      if (syncReadPositions()) {
        for (size_t num = 0; num < _joint_names.size(); num++)
          _current_position[num] += _joint_mounting_offsets[num] + _joint_offsets[num];
      } else
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint position!");
    }

    if (_read_velocity) {
      if (!syncReadVelocities()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint velocity!");
      }
    }

    if (_read_effort) {
      if (!syncReadEfforts()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint effort!");
      }
    }
  }

  if (_read_volt_temp){
    if (_read_VT_counter > _VT_update_rate){
      if(!syncReadVoltageAndTemp()){
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current input volatage and temperature!");
        _read_VT_counter = 0;
      }
    }else{
      _read_VT_counter++;
    }
  }

  if (first_cycle_)
  {
    _goal_position = _current_position;
    first_cycle_ = false;
  }


}

void DynamixelHardwareInterface::write()
{
  if(_onlyIMU){
    return;
  }
  //check if we have to switch the torque
  if(current_torque_ != goal_torque_){
    setTorque(goal_torque_);
  }

  if (_control_mode == PositionControl)
  {
      syncWritePosition();
  } else if (_control_mode == VelocityControl)
  {
      syncWriteVelocity();
  } else if (_control_mode == EffortControl)
  {
      syncWriteCurrent();
  }
}

bool DynamixelHardwareInterface::stringToControlMode(std::string _control_modestr, ControlMode& control_mode)
{
  if (_control_modestr == "position")
  {
    control_mode = PositionControl;
    return true;
  } else if (_control_modestr == "velocity")
  {
    control_mode = VelocityControl;
    return true;
  } else if (_control_modestr == "effort")
  {
    control_mode = EffortControl;
    return true;
  } else {
    return false;
  }
}

bool DynamixelHardwareInterface::switchDynamixelControlMode()
{
  if(_onlyIMU){
    return true;
  }
  // Torque on dynamixels has to be disabled to change operating mode
  setTorque(false);
  ros::Duration(0.5).sleep();

  int32_t value = 3;
  if (_control_mode == PositionControl)
  {
    value = 3;;
  } else if (_control_mode == VelocityControl)
  {
    value = 1;
  } else if (_control_mode == EffortControl)
  {
    value = 0;
  }

  std::vector<int32_t> operating_mode(_joint_names.size(), value);
  int32_t* o = &operating_mode[0];
  _driver->syncWrite("Operating_Mode", o);

  ros::Duration(0.5).sleep();
  //reenable torque
  setTorque(true);
}

bool DynamixelHardwareInterface::syncReadPositions(){
  bool success;
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Position", data);
  for(int i = 0; i < _joint_count; i++){
    _current_position[i] = _driver->convertValue2Radian(_joint_ids[i], data[i]);
  }

  free(data);
  return success;
}

bool DynamixelHardwareInterface::syncReadVelocities(){
  bool success;
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Velocity", data);
  for(int i = 0; i < _joint_count; i++){
    _current_velocity[i] = _driver->convertValue2Velocity(_joint_ids[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelHardwareInterface::syncReadEfforts() {
  bool success; //todo maybe 16bit has to be used in stead, like in the readAll method
  int32_t *data = (int32_t *) malloc(_joint_count * sizeof(int32_t));
  success = _driver->syncRead("Present_Current", data);
  for (int i = 0; i < _joint_count; i++) {
    _current_effort[i] = _driver->convertValue2Torque(_joint_ids[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelHardwareInterface::syncReadVoltageAndTemp(){
  std::vector<uint8_t> data;
  if(_driver->syncReadMultipleRegisters(144, 3, &data)) {
    uint32_t volt;
    uint32_t temp;    
    for (int i = 0; i < _joint_count; i++) {
      volt = DXL_MAKEWORD(data[i * 10], data[i * 10 + 1]);
      temp = data[i * 10 + 2];
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

bool DynamixelHardwareInterface::syncReadAll() {
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
      _current_position[i] = _driver->convertValue2Radian(_joint_ids[i], pos);
    }
    return true;
  }else{
    return false;
  }
}

bool DynamixelHardwareInterface::readImu(){
  uint8_t *data = (uint8_t *) malloc(110 * sizeof(uint8_t));

    if(_driver->readMultipleRegisters(241, 36, 32, data)){
      //todo we have to check if we jumped one sequence number
        uint32_t highest_seq_number = 0;
        uint32_t new_value_index=0;
        uint32_t current_seq_number= 0;
        // imu gives us 2 values at the same time, lets see which one is the newest
        for(int i =0; i < 2; i++){
            //the sequence number are the bytes 12 to 15 for each of the two 16 Bytes
            current_seq_number = DXL_MAKEDWORD(DXL_MAKEWORD(data[16*i+12], data[16*i+13]),
                                             DXL_MAKEWORD(data[16*i+14], data[16*i+15]));
          if(current_seq_number>highest_seq_number){
              highest_seq_number=current_seq_number;
              new_value_index=i;
            }
        }
      // linear acceleration are two signed bytes with 256 LSB per g
      _linear_acceleration[0] = (((short) DXL_MAKEWORD(data[16*new_value_index], data[16*new_value_index+1])) / 256.0 ) * gravity * 1;
      _linear_acceleration[1] = (((short) DXL_MAKEWORD(data[16*new_value_index+2], data[16*new_value_index+3])) / 256.0 ) * gravity * 1;
      _linear_acceleration[2] = (((short)DXL_MAKEWORD(data[16*new_value_index+4], data[16*new_value_index+5])) / 256.0 ) * gravity * -1;
      // angular velocity are two signed bytes with 14.375 per deg/s
      _angular_velocity[0] = (((short)DXL_MAKEWORD(data[16*new_value_index+6], data[16*new_value_index+7])) / 14.375) * M_PI/180 * -1;
      _angular_velocity[1] = (((short)DXL_MAKEWORD(data[16*new_value_index+8], data[16*new_value_index+9])) / 14.375) * M_PI/180 * -1;
      _angular_velocity[2] = (((short)DXL_MAKEWORD(data[16*new_value_index+10], data[16*new_value_index+11])) / 14.375) * M_PI/180 * -1;
      return true;
    }else {
      return false;
    }
}

bool DynamixelHardwareInterface::syncWritePosition(){
  int* goal_position = (int*)malloc(_joint_names.size() * sizeof(int));
  float radian;
  for (size_t num = 0; num < _joint_names.size(); num++) {
    radian = _goal_position[num] - _joint_mounting_offsets[num] - _joint_offsets[num];
    goal_position[num] = _driver->convertRadian2Value(_joint_ids[num], radian);
  }
  _driver->syncWrite("Goal_Position", goal_position);
  free(goal_position);
}

bool DynamixelHardwareInterface::syncWriteVelocity() {
  int* goal_velocity = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    goal_velocity[num] = _driver->convertRadian2Value(_joint_ids[num], _goal_velocity[num]);
  }
  _driver->syncWrite("Goal_Velocity", goal_velocity);
  free(goal_velocity);
}

bool DynamixelHardwareInterface::syncWriteCurrent() {
  int* goal_current = (int*)malloc(_joint_names.size() * sizeof(int));
  for (size_t num = 0; num < _joint_names.size(); num++) {
    goal_current[num] = _driver->convertRadian2Value(_joint_ids[num], _goal_effort[num]);
  }
  _driver->syncWrite("Goal_Current", goal_current);
  free(goal_current);
}

void DynamixelHardwareInterface::reconf_callback(bitbots_ros_control::bitbots_ros_control_paramsConfig &config, uint32_t level) {
  _read_position = config.read_position;
  _read_velocity = config.read_velocity;
  _read_effort = config.read_effort;
  _read_imu = config.read_imu;
  _read_volt_temp = config.read_volt_temp;
  _VT_update_rate = config.VT_update_rate;
}

}
