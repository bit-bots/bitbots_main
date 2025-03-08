#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_utils/utils.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace bitbots_odometry {

class FusedOdometry : public rclcpp::Node {
 public:
  FusedOdometry();
  void loop();

 private:
  nav_msgs::msg::Odometry prev_odom_msg_;
  nav_msgs::msg::Odometry current_odom_msg_;
  sensor_msgs::msg::Imu prev_imu_msg_;
  sensor_msgs::msg::Imu current_imu_msg_;
  tf2::Transform odom_to_base_;
  std::string base_link_frame_, odom_frame_, imu_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

  bool first_imu_msg_received_ = false;
  bool first_odom_msg_received_ = false;
};

}  // namespace bitbots_odometry
