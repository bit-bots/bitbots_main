#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle& nh){

  _speak_pub = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1);

  // load parameters
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  nh.getParam("onlyImu", _onlyImu);
  if(_onlyImu) ROS_WARN("Starting in only IMU mode");
  nh.getParam("onlyPressure", _onlyPressure);
  if(_onlyPressure) ROS_WARN("starting in only pressure sensor mode");

  // init bus driver
  std::string port_name;
  nh.getParam("port_info/port_name", port_name);
  int baudrate;
  nh.getParam("port_info/baudrate", baudrate);
  boost::shared_ptr<DynamixelDriver> driver;
  if(!driver->init(port_name.c_str(), uint32_t(baudrate))){
    ROS_ERROR("Error opening serial port %s", port_name.c_str());
    speak_error(_speak_pub, "Error opening serial port");
    sleep(1);
    exit(1);
  }
  float protocol_version;
  nh.getParam("port_info/protocol_version", protocol_version);
  driver->setPacketHandler(protocol_version);

  _servos = DynamixelServoHardwareInterface(driver);
  _imu = ImuHardwareInterface(driver);
  _left_foot = BitFootHardwareInterface(driver, 101, "/left_foot_pressure");
  _right_foot = BitFootHardwareInterface(driver, 102, "/right_foot_pressure");
  _buttons = ButtonHardwareInterface(driver);

  // set the dynamic reconfigure and load standard params for servo interface
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> server;
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconf_callback,&_servos, _1, _2);
  server.setCallback(f);

}

bool WolfgangHardwareInterface::init(ros::NodeHandle& root_nh){
  bool success = true;
  if(_onlyImu) {
    success &= _imu.init(root_nh);
  }else if(_onlyPressure){
    success &= _left_foot.init(root_nh);
    success &= _right_foot.init(root_nh);
  }else {
    success &= _servos.init(root_nh);
    success &= _imu.init(root_nh);
    success &= _left_foot.init(root_nh);
    success &= _right_foot.init(root_nh);
    success &= _buttons.init(root_nh);
  }
  if(success) {
    speak_error(_speak_pub, "ros control startup successful");
  }else{
    speak_error(_speak_pub, "error starting ros control");
  }
  return success;
}


bool WolfgangHardwareInterface::read()
{
  bool success = true;
  if(_onlyImu){
    success &= _imu.read();
  }else if(_onlyPressure){
    success &= _left_foot.read();
    success &= _right_foot.read();
  }else{
    success &= _servos.read();
    success &= _imu.read();
    success &= _left_foot.read();
    success &= _right_foot.read();
    success &= _buttons.read();
  }
  return success;
}

void WolfgangHardwareInterface::write()
{
  if(!_onlyImu && !_onlyPressure) {
    _servos.write();
  }
}
}
