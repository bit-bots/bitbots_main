#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Time.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>


class OdometryFuser {
 public:
  OdometryFuser();
 private:
  sensor_msgs::Imu _imu_data;
  nav_msgs::Odometry _odom_data;
  ros::Time _imu_update_time;
  ros::Time _odom_update_time;
  geometry_msgs::TransformStamped tf;
  char current_support_state_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void supportCallback(const std_msgs::Char::ConstPtr &msg);
  tf2::Transform getCurrentRotationPoint();
};
