#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Time.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <Eigen/Geometry>
#include <rot_conv/rot_conv.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <bitbots_msgs/SupportState.h>
#include <unistd.h>

class OdometryFuser {
 public:
  OdometryFuser();
 private:
  sensor_msgs::Imu imu_data_;
  nav_msgs::Odometry odom_data_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Time fused_time_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_, rotation_frame_, imu_frame_;

  message_filters::Cache<bitbots_msgs::SupportState> support_state_cache_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &img_msg, const nav_msgs::Odometry::ConstPtr &motion_odom_msg);
  tf2::Quaternion getCurrentMotionOdomYaw(tf2::Quaternion motion_odom_rotation);
  tf2::Quaternion getCurrentImuRotationWithoutYaw(tf2::Quaternion imu_rotation);
  tf2::Transform getCurrentRotationPoint();
};
