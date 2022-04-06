#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_CONTROLLER_HPP_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_DYNAMIXEL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>

namespace dynamixel_controller {

struct JointCommandData {
  int id;
  double pos;
  double vel;
  double acc;
  double cur;
};

class DynamixelController {
 public:
  DynamixelController(rclcpp::Node::SharedPtr nh);

  void starting(const rclcpp::Time &time);
  void update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/);
  bool init();

  std::vector<std::string> joint_names;
  std::vector<hardware_interface::PosVelAccCurJointHandle> joints;
  realtime_tools::RealtimeBuffer<std::vector<JointCommandData>> commands_buffer;
  unsigned int n_joints;

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr sub_command_;
  std::map<std::string, int> joint_map_;
  void commandCb(const bitbots_msgs::msg::JointCommand &command_msg) {
    if (!(command_msg.joint_names.size() == command_msg.positions.size() &&
        command_msg.joint_names.size() == command_msg.velocities.size() &&
        command_msg.joint_names.size() == command_msg.accelerations.size() &&
        command_msg.joint_names.size() == command_msg.max_currents.size())) {
      RCLCPP_ERROR(nh_->get_logger(), "Dynamixel Controller got command with inconsistent array lengths.");
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