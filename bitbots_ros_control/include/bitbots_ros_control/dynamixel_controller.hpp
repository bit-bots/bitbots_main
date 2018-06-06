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

struct JointCommandData
{
  int id;
  double pos;
  double vel;
  double acc;
  double cur;
};  
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
  realtime_tools::RealtimeBuffer<std::vector<JointCommandData>> commands_buffer_;
  unsigned int n_joints_;

private:
  ros::Subscriber sub_command_;
  std::map<std::string, int> _joint_map;
  void commandCB(const bitbots_ros_control::JointCommand& command_msg) {
    //std::cout << ::getpid();
    std::vector<JointCommandData> buf_data;
    for(unsigned int i = 0; i < command_msg.joint_names.size(); i++){
      /*if(_joint_map.find(command_msg.joint_names[i] == _joint_map.end())){
          ROS_WARN("Joint %s in JointCommand message is not known.", command_msg.joint_names[i].c_str());
          continue;
      }*/
      JointCommandData strct;
      strct.id = _joint_map[command_msg.joint_names[i]];
      strct.pos = command_msg.positions[i];
      strct.vel = command_msg.velocities[i];
      strct.acc = command_msg.accelerations[i];
      strct.cur = command_msg.max_currents[i];
      buf_data.push_back(strct);   
    }
    commands_buffer_.writeFromNonRT(buf_data);
  }
};

}

#endif