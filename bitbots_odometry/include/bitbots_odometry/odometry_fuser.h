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
#include <bitbots_msgs/SupportState.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class OdometryFuser {
 public:
  OdometryFuser();
 private:
  sensor_msgs::Imu _imu_data;
  nav_msgs::Odometry _odom_data;
  geometry_msgs::TransformStamped tf;
  char current_support_state_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_, rotation_frame_, imu_frame_, cop_frame_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &img_msg, const nav_msgs::Odometry::ConstPtr &motion_odom_msg);
  void supportCallback(const bitbots_msgs::SupportState msg);
  tf2::Quaternion getCurrentMotionOdomYaw(tf2::Quaternion motion_odom_rotation);
  tf2::Quaternion getCurrentImuRotationWithoutYaw(tf2::Quaternion imu_rotation);
  tf2::Transform getCurrentRotationPoint();
};
