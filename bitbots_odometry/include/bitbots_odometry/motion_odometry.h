#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <bitbots_msgs/msg/support_state.hpp>
#include <unistd.h>
#include <tf2_ros/buffer.h>
using std::placeholders::_1;

class MotionOdometry : public rclcpp::Node {
 public:
  MotionOdometry();
 private:
  rclcpp::Time joint_update_time_;
  char current_support_state_;
  char previous_support_state_;
  rclcpp::Time current_support_state_time_;
  sensor_msgs::msg::JointState current_joint_states_;
  nav_msgs::msg::Odometry current_odom_msg_;
  tf2::Transform odometry_to_support_foot_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_;

  double x_forward_scaling_;
  double x_backward_scaling_;
  double y_scaling_;
  double yaw_scaling_;

  void supportCallback(const bitbots_msgs::msg::SupportState::SharedPtr msg);
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
