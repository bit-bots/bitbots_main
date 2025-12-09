#include <bitbots_modern_rl_motion/walk_node.hpp>

namespace bitbots_modern_rl_motion {

WalkNode::WalkNode(rclcpp::Node::SharedPtr node, const std::string& ns,
                   const std::vector<rclcpp::Parameter>& moveit_parameters)
    : node_(node) {
  pub_controller_command_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("walk_engine_odometry", 1);
  pub_support_ = node_->create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 1);
  step_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("step", 1, std::bind(&WalkNode::stepCb, this, _1));
  cmd_vel_sub_ =
      node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&WalkNode::cmdVelCb, this, _1));
  robot_state_sub_ = node_->create_subscription<bitbots_msgs::msg::RobotControlState>(
      "robot_state", 1, std::bind(&WalkNode::robotStateCb, this, _1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&WalkNode::jointStateCb, this, _1));
  kick_sub_ = node_->create_subscription<std_msgs::msg::Bool>("kick", 1, std::bind(&WalkNode::kickCb, this, _1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&WalkNode::imuCb, this, _1));
  pressure_sub_left_ = node_->create_subscription<bitbots_msgs::msg::FootPressure>(
      "foot_pressure_left/filtered", 1, std::bind(&WalkNode::pressureLeftCb, this, _1));
  pressure_sub_right_ = node_->create_subscription<bitbots_msgs::msg::FootPressure>(
      "foot_pressure_right/filtered", 1, std::bind(&WalkNode::pressureRightCb, this, _1));
}

}  // namespace bitbots_modern_rl_motion
