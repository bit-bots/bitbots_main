#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_ros_control/bitbots_ros_control_paramsConfig.h>
#include <bitbots_ros_control/wolfgang_hardware_interface.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  ros::NodeHandle pnh("~");

  // create hardware interfaces
  bitbots_ros_control::WolfgangHardwareInterface hw = bitbots_ros_control::WolfgangHardwareInterface();

  // set the dynamic reconfigure and load standard params for servo interface
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_hardware_interface_paramsConfig> server;
  dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_hardware_interface_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconf_callback,&hw, _1, _2);
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
