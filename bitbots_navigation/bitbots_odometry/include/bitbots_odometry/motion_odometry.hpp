#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_utils/utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <odometry_parameters.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace bitbots_odometry {

class MotionOdometry : public rclcpp::Node {
 public:
  MotionOdometry();
  void loop();

 private:
  rclcpp::Time joint_update_time_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  int current_support_state_ = -1;
  int previous_support_state_ = -1;
  rclcpp::Time current_support_state_time_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  nav_msgs::msg::Odometry current_odom_msg_;
  tf2::Transform odometry_to_support_foot_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr walk_support_state_sub_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr kick_support_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // Declare parameter listener and struct from the generate_parameter_library
  motion_odometry::ParamListener param_listener_;
  // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  motion_odometry::Params config_;

  void supportCallback(const biped_interfaces::msg::Phase::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Time foot_change_time_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  std::string previous_support_link_;
  std::string current_support_link_;
  rclcpp::Time start_time_;
};

}  // namespace bitbots_odometry
