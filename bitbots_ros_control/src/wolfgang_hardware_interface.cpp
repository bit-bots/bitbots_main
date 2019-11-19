#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle& nh){
  speak_pub_ = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1);

  // load parameters
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  nh.getParam("only_imu", only_imu_);
  if(only_imu_) ROS_WARN("Starting in only IMU mode");
  nh.getParam("only_pressure", only_pressure_);
  if(only_pressure_) ROS_WARN("starting in only pressure sensor mode");

  if(only_pressure_ && only_imu_) {
    ROS_ERROR("only_imu AND only_pressure was set to true");
    exit(1);
  }

  // init bus driver
  std::string port_name;
  nh.getParam("port_info/port_name", port_name);
  int baudrate;
  nh.getParam("port_info/baudrate", baudrate);
  auto driver = std::make_shared<DynamixelDriver>();
  if(!driver->init(port_name.c_str(), uint32_t(baudrate))){
    ROS_ERROR("Error opening serial port %s", port_name.c_str());
    speakError(speak_pub_, "Error opening serial port");
    sleep(1);
    exit(1);
  }
  float protocol_version;
  nh.getParam("port_info/protocol_version", protocol_version);
  driver->setPacketHandler(protocol_version);

  servos_ = DynamixelServoHardwareInterface(driver);
  imu_ = ImuHardwareInterface(driver);
  right_foot_ = BitFootHardwareInterface(driver, 102, "/foot_pressure_right/raw");
  buttons_ = ButtonHardwareInterface(driver);

  // set the dynamic reconfigure and load standard params for servo interface
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> server;
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconfCallback, &servos_, _1, _2);
  server.setCallback(f);

}

bool WolfgangHardwareInterface::init(ros::NodeHandle& root_nh){
  bool success = true;
  if(only_imu_) {
    imu_.setParent(this);
    success &= imu_.init(root_nh);
  }else if(only_pressure_){
    success &= right_foot_.init(root_nh);

  }else {
    /* Hardware interfaces must be registered at the main RobotHW class.
     * Therefore, a pointer to this class is passed down to the RobotHW classes
     * registering further interfaces */
    servos_.setParent(this);
    imu_.setParent(this);
    success &= servos_.init(root_nh);
    success &= imu_.init(root_nh);
    success &= right_foot_.init(root_nh);

    success &= buttons_.init(root_nh);
  }
  if(success) {
    speakError(speak_pub_, "ros control startup successful");
  }else{
    speakError(speak_pub_, "error starting ros control");
  }
  return success;
}


bool WolfgangHardwareInterface::read()
{
  bool success = true;
  if(only_imu_){
    success &= imu_.read();
  }else if(only_pressure_){
    success &= right_foot_.read();

  }else{
    success &= servos_.read();
    success &= imu_.read();
    success &= right_foot_.read();

    success &= buttons_.read();
  }
  return success;
}

void WolfgangHardwareInterface::write()
{
  if(!only_imu_ && !only_pressure_) {
    servos_.write();
  }
}
}
