#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/char.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
#include <biped_interfaces/msg/phase.hpp>
#include <unistd.h>
using std::placeholders::_1;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, nav_msgs::msg::Odometry> SyncPolicy;

class OdometryFuser : public rclcpp::Node {
 public:
  OdometryFuser();
  void loop();
 private:
  sensor_msgs::msg::Imu imu_data_;
  nav_msgs::msg::Odometry odom_data_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time fused_time_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_, rotation_frame_, imu_frame_;

  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr walk_support_state_sub_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr kick_support_state_sub_;

  message_filters::Cache<biped_interfaces::msg::Phase> support_state_cache_;

  void supportCallback(const biped_interfaces::msg::Phase::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr &img_msg, const nav_msgs::msg::Odometry::SharedPtr &motion_odom_msg);
  tf2::Quaternion getCurrentMotionOdomYaw(tf2::Quaternion motion_odom_rotation);
  tf2::Quaternion getCurrentImuRotationWithoutYaw(tf2::Quaternion imu_rotation);
  tf2::Transform getCurrentRotationPoint();
};

