#include <bitbots_odometry/motion_odometry.h>

MotionOdometry::MotionOdometry() : Node("MotionOdometry"), tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())) {
  bool publish_walk_odom_tf;
  this->declare_parameter<bool>("publish_walk_odom_tf", false);
  this->get_parameter("publish_walk_odom_tf", publish_walk_odom_tf);
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame_);
  this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  this->get_parameter("r_sole_frame", r_sole_frame_);
  this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  this->get_parameter("l_sole_frame", l_sole_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);
  this->declare_parameter<double>("x_forward_scaling", 1);
  this->get_parameter("x_forward_scaling", x_forward_scaling_);
  this->declare_parameter<double>("x_backward_scaling", 1);
  this->get_parameter("x_backward_scaling", x_backward_scaling_);
  this->declare_parameter<double>("y_scaling", 1);
  this->get_parameter("y_scaling", y_scaling_);
  this->declare_parameter<double>("yaw_scaling", 1);
  this->get_parameter("yaw_scaling", yaw_scaling_);
  current_support_state_ = -1;
  previous_support_state_ = -1;
  std::string previous_support_link = r_sole_frame_;
  std::string current_support_link;
  rclcpp::Subscription<bitbots_msgs::msg::SupportState>::SharedPtr walk_support_state_sub =
      this->create_subscription<bitbots_msgs::msg::SupportState>("walk_support_state",
                                                                 1,
                                                                 std::bind(&MotionOdometry::supportCallback,
                                                                           this,
                                                                           _1));
  rclcpp::Subscription<bitbots_msgs::msg::SupportState>::SharedPtr kick_support_state_sub =
      this->create_subscription<bitbots_msgs::msg::SupportState>("dynamic_kick_support_state",
                                                                 1,
                                                                 std::bind(&MotionOdometry::supportCallback,
                                                                           this,
                                                                           _1));
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub =
      this->create_subscription<sensor_msgs::msg::JointState>("joint_states",
                                                              1,
                                                              std::bind(&MotionOdometry::jointStateCb, this, _1));
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber =
      this->create_subscription<nav_msgs::msg::Odometry>("walk_engine_odometry",
                                                         1,
                                                         std::bind(&MotionOdometry::odomCallback, this, _1));

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("motion_odometry", 1);
  // set the origin to 0. will be set correctly on recieving first support state
  odometry_to_support_foot_.setOrigin({0, 0, 0});
  odometry_to_support_foot_.setRotation(tf2::Quaternion(0, 0, 0, 1));

  rclcpp::Time foot_change_time;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // wait till connection with publishers has been established
  // so we do not immediately blast something into the log output
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  rclcpp::Rate r(200.0);

  while (rclcpp::ok()) {
    rclcpp::spin_some(std::make_shared<MotionOdometry>());
    if (r.sleep()) {
      //check if joint states were received, otherwise we can't provide odometry
      rclcpp::Duration joints_delta_t = this->now() - joint_update_time_;
      if (joints_delta_t > rclcpp::Duration(0.05 * 1e9)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30, "No joint states received. Will not provide odometry.");
      } else {
        // check if step finished, meaning left->right or right->left support. double support is skipped
        // the support foot change is published when the joint goals for the last movements are published.
        // it takes some time till the joints actually reach this position, this can create some offset
        // but since we skip the double support phase, we basically take the timepoint when the double support phase is
        // finished. This means both feet did not move and this should create no offset.
        if ((current_support_state_ == bitbots_msgs::msg::SupportState::LEFT
            && previous_support_state_ == bitbots_msgs::msg::SupportState::RIGHT) ||
            (current_support_state_ == bitbots_msgs::msg::SupportState::RIGHT
                && previous_support_state_ == bitbots_msgs::msg::SupportState::LEFT)) {
          foot_change_time = current_support_state_time_;
          if (previous_support_state_ == bitbots_msgs::msg::SupportState::LEFT) {
            previous_support_link = l_sole_frame_;
            current_support_link = r_sole_frame_;
          } else {
            previous_support_link = r_sole_frame_;
            current_support_link = l_sole_frame_;
          }

          try {
            // add the transform between previous and current support link to the odometry transform.
            // we wait a bit for the transform as the joint messages are maybe a bit behind
            geometry_msgs::msg::TransformStamped previous_to_current_support_msg =
                tf_buffer_->lookupTransform(previous_support_link,
                                     current_support_link,
                                     foot_change_time,
                                     rclcpp::Duration(0.1));
            tf2::Transform previous_to_current_support = tf2::Transform();
            tf2::fromMsg(previous_to_current_support_msg.transform, previous_to_current_support);
            // setting translation in z axis, pitch and roll to zero to stop the robot from lifting up
            // scale odometry based on parameters
            double x = previous_to_current_support.getOrigin().x();
            if (x > 0) {
              x = x * x_forward_scaling_;
            } else {
              x = x * x_backward_scaling_;
            }
            double y = previous_to_current_support.getOrigin().y() * y_scaling_;
            double yaw = tf2::getYaw(previous_to_current_support.getRotation()) * yaw_scaling_;
            previous_to_current_support.setOrigin({x, y, 0});
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            previous_to_current_support.setRotation(q);
            odometry_to_support_foot_ = odometry_to_support_foot_ * previous_to_current_support;
          } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            continue;
          }

          // update current support link for transform from foot to base link
          previous_support_link = current_support_link;

          // remember the support state change but skip the double support phase
          if (current_support_state_ != bitbots_msgs::msg::SupportState::DOUBLE) {
            previous_support_state_ = current_support_state_;
          }
        }

        //publish odometry and if wanted transform to base_link
        try {
          geometry_msgs::msg::TransformStamped
              current_support_to_base_msg =
              tf_buffer_->lookupTransform(previous_support_link, base_link_frame_, rclcpp::Time(0));
          tf2::Transform current_support_to_base;
          tf2::fromMsg(current_support_to_base_msg.transform, current_support_to_base);
          tf2::Transform odom_to_base_link = odometry_to_support_foot_ * current_support_to_base;
          geometry_msgs::msg::TransformStamped odom_to_base_link_msg = geometry_msgs::msg::TransformStamped();
          odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link);
          odom_to_base_link_msg.header.stamp = current_support_to_base_msg.header.stamp;
          odom_to_base_link_msg.header.frame_id = odom_frame_;
          odom_to_base_link_msg.child_frame_id = base_link_frame_;
          if (publish_walk_odom_tf) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Sending Tf from walk odometry directly");
            br->sendTransform(odom_to_base_link_msg);
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
          pub_odometry->publish(odom_msg);

        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "%s", ex.what());
          rclcpp::sleep_for(std::chrono::milliseconds(100));
          continue;
        }
      }
    } else {
      usleep(1);
    }
  }
}

void MotionOdometry::supportCallback(const bitbots_msgs::msg::SupportState::SharedPtr msg) {
  current_support_state_ = msg->state;
  current_support_state_time_ = msg->header.stamp;

  // remember if we received first support state, only remember left or right
  if (previous_support_state_ == -1 && current_support_state_ != bitbots_msgs::msg::SupportState::DOUBLE) {
    std::string current_support_link;
    if (current_support_state_ == bitbots_msgs::msg::SupportState::LEFT) {
      previous_support_state_ = bitbots_msgs::msg::SupportState::RIGHT;
      current_support_link = l_sole_frame_;
    } else {
      previous_support_state_ = bitbots_msgs::msg::SupportState::LEFT;
      current_support_link = r_sole_frame_;
    }
    // on receiving first support state we should also set the location in the world correctly
    // we assume that our baseline is on x=0 and y=0
    try {
      geometry_msgs::msg::TransformStamped
          base_to_current_support_msg =
          tf_buffer_->lookupTransform(base_link_frame_, current_support_link, rclcpp::Time(0), rclcpp::Duration(10.0));
      odometry_to_support_foot_.setOrigin({-1 * base_to_current_support_msg.transform.translation.x,
                                           -1 * base_to_current_support_msg.transform.translation.y, 0});
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not initialize motion odometry correctly, since there were no transforms available fast enough on startup. Will initialize with 0,0,0");
    }
  }
}

void MotionOdometry::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  current_joint_states_ = *msg;
  joint_update_time_ = this->now();
}

void MotionOdometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_msg_ = *msg;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  MotionOdometry o;
}
