#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_CONTROLLER_HPP_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_CONTROLLER_HPP_

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_ros_control/posvelacccur_command_interface.h>

namespace dynamixel_controller {

struct JointCommandData {
  int id;
  double pos;
  double vel;
  double acc;
  double cur;
};
class DynamixelController : public controller_interface::Controller<hardware_interface::PosVelAccCurJointInterface> {
 public:
  DynamixelController() = default;
  ~DynamixelController() override { sub_command_.shutdown(); }

  void starting(const ros::Time &time) override;
  void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;
  bool init(hardware_interface::PosVelAccCurJointInterface *hw, ros::NodeHandle &n) override;

  std::vector<std::string> joint_names;
  std::vector<hardware_interface::PosVelAccCurJointHandle> joints;
  realtime_tools::RealtimeBuffer<std::vector<JointCommandData>> commands_buffer;
  unsigned int n_joints;

 private:
  ros::Subscriber sub_command_;
  std::map<std::string, int> joint_map_;
  void commandCb(const bitbots_msgs::JointCommand &command_msg) {
    if (!(command_msg.joint_names.size() == command_msg.positions.size() &&
        command_msg.joint_names.size() == command_msg.velocities.size() &&
        command_msg.joint_names.size() == command_msg.accelerations.size() &&
        command_msg.joint_names.size() == command_msg.max_currents.size())) {
      ROS_ERROR("Dynamixel Controller got command with inconsistent array lengths.");
      return;
    }
    std::vector<JointCommandData> buf_data;
    for (unsigned int i = 0; i < command_msg.joint_names.size(); i++) {
      JointCommandData strct;
      strct.id = joint_map_[command_msg.joint_names[i]];
      strct.pos = command_msg.positions[i];
      strct.vel = command_msg.velocities[i];
      strct.acc = command_msg.accelerations[i];
      strct.cur = command_msg.max_currents[i];
      buf_data.push_back(strct);
    }
    commands_buffer.writeFromNonRT(buf_data);
  }
};

}

#endif