#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <humanoid_league_msgs/PoseWithCertainty.h>
#include <humanoid_league_msgs/PoseWithCertaintyArray.h>
#include <bitbots_msgs/SupportState.h>

#include <utility>

class ConvenienceFramesBroadcaster {
 public:
  ConvenienceFramesBroadcaster();
 private:
  tf2_ros::TransformBroadcaster broadcaster_{tf2_ros::TransformBroadcaster()};
  geometry_msgs::TransformStamped tf_{geometry_msgs::TransformStamped()};
  tf2_ros::Buffer tfBuffer_{ros::Duration(1.0)};
  tf2_ros::TransformListener tfListener_{tfBuffer_};
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, r_toe_frame_, l_toe_frame_, approach_frame_,
              ball_frame_, right_post_frame_, left_post_frame_, general_post_frame_;

  bool is_left_support{false};
  bool got_support_foot_{false};
  void publishTransform(std::string header_frame_id, std::string child_frame_id, double x, double y, double z);
  void supportFootCallback(const bitbots_msgs::SupportState msg);
  void ballsCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg);
  void goalCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg);
  void goalPostsCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg);
};
