#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <humanoid_league_msgs/msg/pose_with_certainty.hpp>
#include <humanoid_league_msgs/msg/pose_with_certainty_array.hpp>
#include <biped_interfaces/msg/phase.hpp>

#include <utility>
using std::placeholders::_1;

class ConvenienceFramesBroadcaster : public rclcpp::Node {
 public:
  ConvenienceFramesBroadcaster();
  void loop();
 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped tf_{geometry_msgs::msg::TransformStamped()};
  std::unique_ptr<tf2_ros::Buffer>
      tfBuffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())};
  tf2_ros::TransformListener tfListener_{*tfBuffer_, this};
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, r_toe_frame_, l_toe_frame_, approach_frame_,
      ball_frame_, right_post_frame_, left_post_frame_, general_post_frame_;

  bool is_left_support{false};
  bool got_support_foot_{false};
  void publishTransform(std::string header_frame_id, std::string child_frame_id, double x, double y, double z);
  void supportFootCallback(const biped_interfaces::msg::Phase::SharedPtr msg);
  void ballsCallback(const humanoid_league_msgs::msg::PoseWithCertaintyArray::SharedPtr msg);
  void goalCallback(const humanoid_league_msgs::msg::PoseWithCertaintyArray::SharedPtr msg);
  void goalPostsCallback(const humanoid_league_msgs::msg::PoseWithCertaintyArray::SharedPtr msg);
};
