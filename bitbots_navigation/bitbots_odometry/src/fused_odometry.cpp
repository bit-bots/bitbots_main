#include <bitbots_odometry/fused_odometry.hpp>

namespace bitbots_odometry {

FusedOdometry::FusedOdometry()
    : Node("FusedOdometry"),
      odom_to_base_(
          tf2::Transform::getIdentity()),  // TODO maybe identity is not completly correct here, because z-axis
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_, this),
      br_(std::make_unique<tf2_ros::TransformBroadcaster>(this)) {
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);
  this->declare_parameter<std::string>("imu_frame", "imu_frame");
  this->get_parameter("imu_frame", imu_frame_);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("motion_odometry", 1,
                                                                 std::bind(&FusedOdometry::odomCallback, this, _1));
  imu_sub_ =
      this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&FusedOdometry::IMUCallback, this, _1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("fused_odometry", 1);
}

void FusedOdometry::loop() {
  // Wait for tf to be available
  bitbots_utils::wait_for_tf(this->get_logger(), this->get_clock(), &tf_buffer_, {imu_frame_}, base_link_frame_);

  try {
    if (!first_odom_msg_received_ || !first_imu_msg_received_) {
      return;
    }

    nav_msgs::msg::Odometry current_odom_msg = current_odom_msg_;

    tf2::Transform previous_motion_odom = tf2::Transform();
    tf2::Transform current_motion_odom = tf2::Transform();

    previous_motion_odom.setOrigin(tf2::Vector3(
        prev_odom_msg_.pose.pose.position.x, prev_odom_msg_.pose.pose.position.y, prev_odom_msg_.pose.pose.position.z));
    previous_motion_odom.setRotation(
        tf2::Quaternion(prev_odom_msg_.pose.pose.orientation.x, prev_odom_msg_.pose.pose.orientation.y,
                        prev_odom_msg_.pose.pose.orientation.z, prev_odom_msg_.pose.pose.orientation.w));
    current_motion_odom.setOrigin(tf2::Vector3(current_odom_msg.pose.pose.position.x,
                                               current_odom_msg.pose.pose.position.y,
                                               current_odom_msg.pose.pose.position.z));
    current_motion_odom.setRotation(
        tf2::Quaternion(current_odom_msg.pose.pose.orientation.x, current_odom_msg.pose.pose.orientation.y,
                        current_odom_msg.pose.pose.orientation.z, current_odom_msg.pose.pose.orientation.w));

    tf2::Transform previous_to_current_motion_odom = tf2::Transform();
    previous_to_current_motion_odom = previous_motion_odom.inverseTimes(current_motion_odom);

    sensor_msgs::msg::Imu current_imu_msg = current_imu_msg_;

    tf2::Transform previous_imu = tf2::Transform();
    tf2::Transform current_imu = tf2::Transform();
    previous_imu.setOrigin(
        tf2::Vector3(prev_imu_msg_.orientation.x, prev_imu_msg_.orientation.y, prev_imu_msg_.orientation.z));
    previous_imu.setRotation(tf2::Quaternion(prev_imu_msg_.orientation.x, prev_imu_msg_.orientation.y,
                                             prev_imu_msg_.orientation.z, prev_imu_msg_.orientation.w));
    current_imu.setOrigin(
        tf2::Vector3(current_imu_msg.orientation.x, current_imu_msg.orientation.y, current_imu_msg.orientation.z));
    current_imu.setRotation(tf2::Quaternion(current_imu_msg.orientation.x, current_imu_msg.orientation.y,
                                            current_imu_msg.orientation.z, current_imu_msg.orientation.w));

    tf2::Transform imu_mounting_offset;
    try {
      geometry_msgs::msg::TransformStamped imu_mounting_transform =
          tf_buffer_.lookupTransform(current_imu_msg.header.frame_id, base_link_frame_, current_imu_msg.header.stamp);
      fromMsg(imu_mounting_transform.transform, imu_mounting_offset);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Not able to fuse IMU data with odometry due to a tf problem: %s", ex.what());
    }

    tf2::Transform prev_imu_mounting_offset;
    try {
      geometry_msgs::msg::TransformStamped prev_imu_mounting_transform =
          tf_buffer_.lookupTransform(prev_imu_msg_.header.frame_id, base_link_frame_, current_imu_msg_.header.stamp);
      fromMsg(prev_imu_mounting_transform.transform, prev_imu_mounting_offset);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Not able to fuse IMU data with odometry due to a tf problem: %s", ex.what());
    }

    tf2::Transform previous_to_current_imu = tf2::Transform();
    previous_to_current_imu = (previous_imu * prev_imu_mounting_offset).inverseTimes(current_imu * imu_mounting_offset);

    tf2::Transform prev_to_curr_odom = tf2::Transform();
    double yaw = tf2::getYaw(previous_to_current_imu.getRotation());
    prev_to_curr_odom.setOrigin(
        {previous_to_current_motion_odom.getOrigin().x(), previous_to_current_motion_odom.getOrigin().y(), 0});
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    prev_to_curr_odom.setRotation(q);

    odom_to_base_ = odom_to_base_ * prev_to_curr_odom;

    prev_odom_msg_ = current_odom_msg;
    prev_imu_msg_ = current_imu_msg;

    geometry_msgs::msg::TransformStamped odom_to_base_link_msg = geometry_msgs::msg::TransformStamped();
    odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_);
    odom_to_base_link_msg.header.stamp = current_odom_msg.header.stamp;  // TODO: maybe use right now as stamp
    odom_to_base_link_msg.header.frame_id = odom_frame_;
    odom_to_base_link_msg.child_frame_id = base_link_frame_;

    // odometry as message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_odom_msg.header.stamp;  // TODO: maybe use right now as stamp
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = odom_to_base_link_msg.transform.translation.x;
    odom_msg.pose.pose.position.y = odom_to_base_link_msg.transform.translation.y;
    odom_msg.pose.pose.position.z = odom_to_base_link_msg.transform.translation.z;
    odom_msg.pose.pose.orientation = odom_to_base_link_msg.transform.rotation;
    odom_msg.twist = current_odom_msg_.twist;
    odom_pub_->publish(odom_msg);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return;
  }
}

void FusedOdometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!first_odom_msg_received_) {
    prev_odom_msg_ = *msg;
    first_odom_msg_received_ = true;
  }
  current_odom_msg_ = *msg;
}

void FusedOdometry::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!first_imu_msg_received_) {
    prev_imu_msg_ = *msg;
    first_imu_msg_received_ = true;
  }
  current_imu_msg_ = *msg;
}

}  // namespace bitbots_odometry

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_odometry::FusedOdometry>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 200.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });

  exec.spin();
  rclcpp::shutdown();
}
