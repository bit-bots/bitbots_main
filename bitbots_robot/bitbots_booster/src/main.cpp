#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>

#include <booster_interface/msg/odometer.hpp>

#include "utils.h"

#include <rclcpp/rclcpp.hpp>


using std::placeholders::_1;

class BitbotsBooster : public rclcpp::Node {
 public:
  BitbotsBooster() : Node("bitbots_booster") {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Publisher
    motion_pub = this->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 10);

    // Subscribers
    head_transform_sub = this->create_subscription<geometry_msgs::msg::Pose>(
      "/head_pose", 1, std::bind(&BitbotsBooster::head_transform_callback, this, _1));
    odom_transform_sub = this->create_subscription<booster_interface::msg::Odometer>(
      "/odometer_state", 1, std::bind(&BitbotsBooster::odom_transform_callback, this, _1));
    cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, std::bind(&BitbotsBooster::cmd_vel_callback, this, _1));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg) {
    booster_msgs::msg::RpcReqMsg output_msg;
    if (msg.angular.x == -1) {
      output_msg = booster_msgs::CreateMoveMsg(0,0,0);
    } else {
      output_msg = booster_msgs::CreateMoveMsg(msg.linear.x, msg.linear.y, msg.angular.z);
    }
    motion_pub->publish(output_msg);
  }

  void odom_transform_callback(const booster_interface::msg::Odometer &msg) {
    geometry_msgs::msg::TransformStamped odom_transform;

    odom_transform.header.stamp = this->get_clock()->now();
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_footprint";
    odom_transform.transform.translation.x = msg.x;
    odom_transform.transform.translation.y = msg.y;

    auto q = tf2::Quaternion();
    q.setRPY(0, 0, msg.theta);
    odom_transform.transform.rotation.x = q.x();
    odom_transform.transform.rotation.y = q.y();
    odom_transform.transform.rotation.z = q.z();
    odom_transform.transform.rotation.w = q.w();

    broadcaster_->sendTransform(odom_transform);
  }

  void head_transform_callback(const geometry_msgs::msg::Pose &msg) {
    // Bring it into the camera transform
    tf2::Matrix3x3 camera_to_camera_optical_frame (
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0
    );

    tf2::Transform base_footprint_to_camera;
    base_footprint_to_camera.setOrigin(tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
    base_footprint_to_camera.setRotation(tf2::Quaternion( msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w));

    tf2::Transform base_footprint_to_camera_optical = base_footprint_to_camera * tf2::Transform(camera_to_camera_optical_frame);

    geometry_msgs::msg::TransformStamped camera_optical_frame_transform;

    camera_optical_frame_transform.header.stamp = this->get_clock()->now();
    camera_optical_frame_transform.header.frame_id = "base_footprint";
    camera_optical_frame_transform.child_frame_id = "camera_color_optical_frame";
    camera_optical_frame_transform.transform.translation.x = base_footprint_to_camera_optical.getOrigin().getX();
    camera_optical_frame_transform.transform.translation.y = base_footprint_to_camera_optical.getOrigin().getY();
    camera_optical_frame_transform.transform.translation.z = base_footprint_to_camera_optical.getOrigin().getZ();

    camera_optical_frame_transform.transform.rotation.x = base_footprint_to_camera_optical.getRotation().getX();
    camera_optical_frame_transform.transform.rotation.y = base_footprint_to_camera_optical.getRotation().getY();
    camera_optical_frame_transform.transform.rotation.z = base_footprint_to_camera_optical.getRotation().getZ();
    camera_optical_frame_transform.transform.rotation.w = base_footprint_to_camera_optical.getRotation().getW();

    broadcaster_->sendTransform(camera_optical_frame_transform);
  }


 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // Publishers
  rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr motion_pub;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr head_transform_sub;
  rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odom_transform_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BitbotsBooster>();

  rclcpp::spin(node);
}
