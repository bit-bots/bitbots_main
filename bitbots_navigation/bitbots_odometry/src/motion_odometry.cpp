#include <bitbots_odometry/motion_odometry.hpp>

namespace bitbots_odometry {

MotionOdometry::MotionOdometry()
    : Node("MotionOdometry"),
      odometry_to_support_foot_(tf2::Transform::getIdentity()),
      param_listener_(get_node_parameters_interface()),
      config_(param_listener_.get_params()),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_, this),
      br_(std::make_unique<tf2_ros::TransformBroadcaster>(this)) {
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame_);
  this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  this->get_parameter("r_sole_frame", r_sole_frame_);
  this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  this->get_parameter("l_sole_frame", l_sole_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);
  this->declare_parameter<std::string>("imu_frame", "imu_frame");
  this->get_parameter("imu_frame", imu_frame_);

  walk_support_state_sub_ = this->create_subscription<biped_interfaces::msg::Phase>(
      "walk_support_state", 1, std::bind(&MotionOdometry::supportCallback, this, _1));
  kick_support_state_sub_ = this->create_subscription<biped_interfaces::msg::Phase>(
      "dynamic_kick_support_state", 1, std::bind(&MotionOdometry::supportCallback, this, _1));
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "walk_engine_odometry", 1, std::bind(&MotionOdometry::odomCallback, this, _1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1,
                                                                     std::bind(&MotionOdometry::IMUCallback, this, _1));

  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("motion_odometry", 1);

  previous_support_link_ = r_sole_frame_;
  start_time_ = this->now();
}

void MotionOdometry::loop() {
  // Wait for tf to be available
  bitbots_utils::wait_for_tf(this->get_logger(), this->get_clock(), &tf_buffer_,
                             {base_link_frame_, r_sole_frame_, l_sole_frame_}, base_link_frame_);

  rclcpp::Time cycle_start_time = this->now();
  config_ = param_listener_.get_params();

  // check if step finished, meaning left->right or right->left support. double support is skipped
  // the support foot change is published when the joint goals for the last movements are published.
  // it takes some time till the joints actually reach this position, this can create some offset
  // but since we skip the double support phase, we basically take the timepoint when the double support phase is
  // finished. This means both feet did not move and this should create no offset.
  if ((current_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE &&
       previous_support_state_ == biped_interfaces::msg::Phase::RIGHT_STANCE) ||
      (current_support_state_ == biped_interfaces::msg::Phase::RIGHT_STANCE &&
       previous_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE)) {
    foot_change_time_ = current_support_state_time_;
    if (previous_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE) {
      previous_support_link_ = l_sole_frame_;
      current_support_link_ = r_sole_frame_;
    } else {
      previous_support_link_ = r_sole_frame_;
      current_support_link_ = l_sole_frame_;
    }

    try {
      // add the transform between previous and current support link to the odometry transform.
      // we wait a bit for the transform as the joint messages are maybe a bit behind
      geometry_msgs::msg::TransformStamped previous_to_current_support_msg =
          tf_buffer_.lookupTransform(previous_support_link_, current_support_link_, foot_change_time_,
                                     rclcpp::Duration::from_nanoseconds(0.1 * 1e9));
      tf2::Transform previous_to_current_support = tf2::Transform();
      tf2::fromMsg(previous_to_current_support_msg.transform, previous_to_current_support);
      // setting translation in z axis, pitch and roll to zero to stop the robot from lifting up
      // scale odometry based on parameters
      double x = previous_to_current_support.getOrigin().x();
      if (x > 0) {
        x = x * config_.x_forward_scaling;
      } else {
        x = x * config_.x_backward_scaling;
      }
      double y = previous_to_current_support.getOrigin().y() * config_.y_scaling;
      double yaw = tf2::getYaw(previous_to_current_support.getRotation()) * config_.yaw_scaling;
      previous_to_current_support.setOrigin({x, y, 0});

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      previous_to_current_support.setRotation(q);

      odometry_to_support_foot_ = odometry_to_support_foot_ * previous_to_current_support;

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      return;
    }

    // update current support link for transform from foot to base link
    previous_support_link_ = current_support_link_;

    // remember the support state change but skip the double support phase
    if (current_support_state_ != biped_interfaces::msg::Phase::DOUBLE_STANCE) {
      previous_support_state_ = current_support_state_;
    }
  }

  // publish odometry and if wanted transform to base_link
  try {
    geometry_msgs::msg::TransformStamped current_support_to_base_msg =
        tf_buffer_.lookupTransform(previous_support_link_, base_link_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME));
    geometry_msgs::msg::TransformStamped imu_to_previous_support_msg =
        tf_buffer_.lookupTransform(previous_support_link_, imu_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME));

    tf2::Transform current_support_to_base;
    tf2::fromMsg(current_support_to_base_msg.transform, current_support_to_base);
    double x = current_support_to_base.getOrigin().x();
    if (current_odom_msg_.twist.twist.linear.x > 0) {
      x = x * config_.x_forward_scaling;
    } else {
      x = x * config_.x_backward_scaling;
    }
    double y = current_support_to_base.getOrigin().y() * config_.y_scaling;
    double yaw = tf2::getYaw(current_support_to_base.getRotation()) * config_.yaw_scaling;
    current_support_to_base.setOrigin({x, y, current_support_to_base.getOrigin().z()});
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    current_support_to_base.setRotation(q);

    tf2::Quaternion imu_rotation;
    tf2::Transform imu_to_previous_support;
    fromMsg(imu_to_previous_support_msg.transform, imu_to_previous_support);

    imu_rotation = current_imu_orientation_ * imu_to_previous_support.getRotation();  // * initial_imu_transform_ *

    tf2::Transform odometry_to_support_foot_with_imu_yaw = odometry_to_support_foot_;
    odometry_to_support_foot_with_imu_yaw.setRotation(imu_rotation);

    tf2::Transform odom_to_base_link = odometry_to_support_foot_with_imu_yaw * current_support_to_base;
    geometry_msgs::msg::TransformStamped odom_to_base_link_msg = geometry_msgs::msg::TransformStamped();
    odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link);
    odom_to_base_link_msg.header.stamp = current_support_to_base_msg.header.stamp;
    odom_to_base_link_msg.header.frame_id = odom_frame_;
    odom_to_base_link_msg.child_frame_id = base_link_frame_;
    if (config_.publish_walk_odom_tf) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Sending Tf from walk odometry directly");
      br_->sendTransform(odom_to_base_link_msg);
    }

    // odometry as message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_support_to_base_msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = odom_to_base_link_msg.transform.translation.x;
    odom_msg.pose.pose.position.y = odom_to_base_link_msg.transform.translation.y;
    odom_msg.pose.pose.position.z = odom_to_base_link_msg.transform.translation.z;
    odom_msg.pose.pose.orientation = odom_to_base_link_msg.transform.rotation;
    odom_msg.twist = current_odom_msg_.twist;
    pub_odometry_->publish(odom_msg);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return;
  }
}

void MotionOdometry::supportCallback(const biped_interfaces::msg::Phase::SharedPtr msg) {
  current_support_state_ = msg->phase;
  current_support_state_time_ = msg->header.stamp;

  // remember if we received first support state, only remember left or right
  if (previous_support_state_ == -1 && current_support_state_ != biped_interfaces::msg::Phase::DOUBLE_STANCE) {
    std::string current_support_link;
    if (current_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE) {
      previous_support_state_ = biped_interfaces::msg::Phase::RIGHT_STANCE;
      current_support_link = l_sole_frame_;
    } else {
      previous_support_state_ = biped_interfaces::msg::Phase::LEFT_STANCE;
      current_support_link = r_sole_frame_;
    }
    // on receiving first support state we should also set the location in the world correctly
    // we assume that our baseline is on x=0 and y=0
    try {
      geometry_msgs::msg::TransformStamped base_to_current_support_msg =
          tf_buffer_.lookupTransform(base_link_frame_, current_support_link, rclcpp::Time(0, 0, RCL_ROS_TIME),
                                     rclcpp::Duration::from_nanoseconds(1e9));
      odometry_to_support_foot_.setOrigin({-1 * base_to_current_support_msg.transform.translation.x,
                                           -1 * base_to_current_support_msg.transform.translation.y, 0});
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not initialize motion odometry correctly, since there were no transforms available fast "
                  "enough on startup. Will initialize with 0,0,0");
    }
  }
}

void MotionOdometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom_msg_ = *msg; }

void MotionOdometry::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::fromMsg(msg->orientation, current_imu_orientation_);
  if (is_initial_transform_set_ == false) {
    is_initial_transform_set_ = true;
    initial_imu_transform_ = current_imu_orientation_;
    initial_imu_transform_.setW(-1.0);
    initial_imu_transform_.normalize();
  }
}

}  // namespace bitbots_odometry

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_odometry::MotionOdometry>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 200.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });

  exec.spin();
  rclcpp::shutdown();
}
