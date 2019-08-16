
#include "bitbots_ros_control/wolfgang_hardware_interface.h"


namespace bitbots_ros_control {
/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle& root_nh) {

  // init bus driver
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  std::string port_name;
  nh.getParam("dynamixels/port_info/port_name", port_name);
  int baudrate;
  nh.getParam("dynamixels/port_info/baudrate", baudrate);
  boost::shared_ptr<DynamixelDriver> driver;
  if(!driver->init(port_name.c_str(), uint32_t(baudrate))){
    ROS_ERROR("Error opening serial port %s", port_name.c_str());
    speak("Error opening serial port");
    sleep(1);
    exit(1);
  }
  float protocol_version;
  nh.getParam("dynamixels/port_info/protocol_version", protocol_version);
  driver->setPacketHandler(protocol_version);

  _servos = DynamixelServoHardwareInterface(driver)
  _imu = ImuHardwareInterface(driver)
  _left_foot = BitFootHardwareInterface(driver, 101, "/left_foot_pressure")
  _right_foot = BitFootHardwareInterface(driver, 102, "/right_foot_pressure")
  _buttons = ButtonHardwareInterface(driver)

}

bool WolfgangHardwareInterface::init(ros::NodeHandle& root_nh){
  _servos.init(root_nh);
  _imu.init(root_nh);
  _left_foot.init(root_nh);
  _right_foot.init(root_nh);
  _buttons.init(root_nh);
}


void WolfgangHardwareInterface::read()
{
  _servos.read();
  _imu.read();
  _left_foot.read();
  _right_foot.read();
  _buttons.read();
}

void WolfgangHardwareInterface::write()
{
  _servos.write();
}
}