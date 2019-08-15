#include <bitbots_ros_control/dynamixel_hardware_interface.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_ros_control/bitbots_ros_control_paramsConfig.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  ros::NodeHandle pnh("~");

  // init driver
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

  // create hardware interfaces
  bitbots_ros_control::DynamixelHardwareInterface hw = bitbots_ros_control::DynamixelHardwareInterface(driver);

  // set the dynamic reconfigure and load standard params
  dynamic_reconfigure::Server<bitbots_ros_control::bitbots_ros_control_paramsConfig> server;
  dynamic_reconfigure::Server<bitbots_ros_control::bitbots_ros_control_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelHardwareInterface::reconf_callback,&hw, _1, _2);
  server.setCallback(f);

  if (!hw.init(pnh))
  {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 1;
  }

  speak("ros control startup successful");

  // Create separate queue, because otherwise controller manager will freeze
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  controller_manager::ControllerManager cm(&hw, nh);

  // Start control loop
  ros::Time current_time = ros::Time::now();
  bool first_update = true;
  ros::Rate rate(pnh.param("control_loop_hz", 200));

  while (ros::ok())
  {    
    bool read_sucessfull = hw.read();
    ros::Duration period = ros::Time::now() - current_time;
    current_time = ros::Time::now(); 
    if(read_sucessfull){ 
      // only write something to hardware 
      if (first_update) {
        first_update = false;
      } else {
        cm.update(current_time, period);
      }
      hw.write();
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
