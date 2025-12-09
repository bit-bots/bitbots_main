#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_msgs/msg/robot_control_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace bitbots_modern_rl_motion {

class WalkNode {
 public:
  explicit WalkNode(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                    const std::vector<rclcpp::Parameter> &moveit_parameters = {});

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_support_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr step_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::RobotControlState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_left_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_right_;
};

}  // namespace bitbots_modern_rl_motion
