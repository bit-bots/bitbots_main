#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <bitbots_msgs/SupportState.h>

class MotionOdometry {
 public:
  MotionOdometry();
 private:
  ros::Time joint_update_time_;
  char current_support_state_;
  char previous_support_state_;
  ros::Time current_support_state_time_;
  sensor_msgs::JointState current_joint_states_;
  nav_msgs::Odometry current_odom_msg_;
  tf2::Transform odometry_to_support_foot_;
  tf2_ros::Buffer tf_buffer_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_;

  double x_forward_scaling_;
  double x_backward_scaling_;
  double y_scaling_;
  double yaw_scaling_;

  void supportCallback(bitbots_msgs::SupportState msg);
  void jointStateCb(const sensor_msgs::JointState &msg);
  void odomCallback(nav_msgs::Odometry msg);
};
