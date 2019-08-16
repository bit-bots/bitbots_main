#include "bitbots_ros_control/wolfgang_hardware_interface.h"


namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle& nh){

  _speak_pub = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1, this);

  // init bus driver
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  std::string port_name;
  nh.getParam("dynamixels/port_info/port_name", port_name);
  int baudrate;
  nh.getParam("dynamixels/port_info/baudrate", baudrate);
  boost::shared_ptr<DynamixelDriver> driver;
  if(!driver->init(port_name.c_str(), uint32_t(baudrate))){
    ROS_ERROR("Error opening serial port %s", port_name.c_str());
    speak_error(_speak_pub, "Error opening serial port");
    sleep(1);
    exit(1);
  }
  float protocol_version;
  nh.getParam("dynamixels/port_info/protocol_version", protocol_version);
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
  bool sucess = true;
  sucess = sucess && _servos.init(root_nh);
  sucess = sucess && _imu.init(root_nh);
  sucess = sucess && _left_foot.init(root_nh);
  sucess = sucess && _right_foot.init(root_nh);
  sucess = sucess && _buttons.init(root_nh);
  if(sucess) {
    speak_error(_speak_pub, "ros control startup successful");
  }else{
    speak_error(_speak_pub, "error starting ros control");
  }

}


bool WolfgangHardwareInterface::read()
{
  bool sucess = true;
  sucess = sucess && _servos.read();
  sucess = sucess && _imu.read();
  sucess = sucess && _left_foot.read();
  sucess = sucess && _right_foot.read();
  sucess = sucess && _buttons.read();
  return sucess;
}

void WolfgangHardwareInterface::write()
{
  _servos.write();
}
}