#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <bitbots_ros_control/JointCommand.h>
#include <bitbots_ros_control/posvelacccur_command_interface.h>

namespace dynamixel_controller
{

class DynamixelController: public controller_interface::Controller<hardware_interface::PosVelAccCurJointInterface>
{
public:
  DynamixelController() {}
  ~DynamixelController() {sub_command_.shutdown();}

  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
  bool init(hardware_interface::PosVelAccCurJointInterface* hw, ros::NodeHandle &n);


  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::PosVelAccCurJointHandle > joints_;
  realtime_tools::RealtimeBuffer<bitbots_ros_control::JointCommand> commands_buffer_;
  unsigned int n_joints_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const bitbots_ros_control::JointCommand& msg) {
    commands_buffer_.writeFromNonRT(msg);
  }
};

}

#endif