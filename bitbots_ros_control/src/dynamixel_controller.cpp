#include <bitbots_ros_control/dynamixel_controller.hpp>

namespace dynamixel_controller {
DynamixelController::DynamixelController(rclcpp::Node::SharedPtr nh) {
  nh_ = nh;
}

bool DynamixelController::init() {
  // Get list of controlled joints from paramserver
  std::string param_name = "joints";
  if (!nh_->get_parameter(param_name, joint_names)) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to getParam '%s' (namespace: %s).", (param_name, nh_->get_namespace()));
    return false;
  }
  n_joints = joint_names.size();

  if (n_joints == 0) {
    RCLCPP_ERROR(nh_->get_logger(), "List of joint names is empty.");
    return false;
  }
  // get handles for joints
  for (unsigned int i = 0; i < n_joints; i++) {
    joints.push_back(hw->getHandle(joint_names[i]));
    joint_map_[joint_names[i]] = i;
  }

  sub_command_ = nh_->create_subscription<bitbots_msgs::msg::JointCommand>("command",
                                                                           1,
                                                                           std::bind(&DynamixelController::commandCb,
                                                                                     this,
                                                                                     std::placeholders::_1));
  return true;
}

void DynamixelController::starting(const rclcpp::Time &time) {}
void DynamixelController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  std::vector<JointCommandData> &buf_data = *commands_buffer.readFromRT();

  // set command for all registered joints
  for (unsigned int i = 0; i < buf_data.size(); i++) {
    joints[buf_data[i].id].setCommand(buf_data[i].pos, buf_data[i].vel, buf_data[i].acc, buf_data[i].cur);
  }
}
}
