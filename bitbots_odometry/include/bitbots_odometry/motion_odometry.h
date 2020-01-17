#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>


class MotionOdometry {
 public:
  MotionOdometry();
 private:
  ros::Time joint_update_time_;
  char current_support_state_;
  char previous_support_state_;
  sensor_msgs::JointState current_joint_states_;
  nav_msgs::Odometry current_odom_msg_;

  void supportCallback(std_msgs::Char msg);
  void jointStateCb(const sensor_msgs::JointState &msg);
  void odomCallback(nav_msgs::Odometry msg);
};
