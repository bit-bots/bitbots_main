#include <rot_conv/rot_conv.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_odometry/odometry_parameters.hpp>
#include <bitbots_utils/utils.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace bitbots_odometry {

class MotionOdometry : public rclcpp::Node {
 public:
  MotionOdometry();
  void loop();

 private:
  int previous_support_state_ =
      biped_interfaces::msg::Phase::LEFT_STANCE;  // Start in left support, as the robot starts standing on the left
                                                  // foot after the dynup
  tf2::Transform odometry_to_support_foot_, odom_to_base_link_, previous_odom_to_base_link_internal_;
  tf2::Quaternion previous_imu_orientation_in_base_link_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_, imu_frame_;
  std::string r_front_left_corner_frame_, r_front_right_corner_frame_, r_back_left_corner_frame_,
      r_back_right_corner_frame_;
  std::string l_front_left_corner_frame_, l_front_right_corner_frame_, l_back_left_corner_frame_,
      l_back_right_corner_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_support_state_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  sensor_msgs::msg::Imu::SharedPtr imu_data_;

  motion_odometry::ParamListener param_listener_;
  motion_odometry::Params config_;

  // Returns {lowest_corner_l, lowest_corner_r, support_state}, where support_state indicates the foot that is lower right now
  std::tuple<std::string, std::string, int> detectSupportFoot(const tf2::Quaternion& imu_rotation);
  // Returns the corrected pose of sole_frame in the IMU frame, as if the foot were flat on the ground
  // pivoted around contact_corner_frame (which is assumed to be the fixed ground contact point).
  tf2::Transform getFootContactModelResult(const tf2::Quaternion& imu_orientation,
                                           const std::string& sole_frame, const std::string& contact_corner_frame);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  tf2::Quaternion dropYaw(tf2::Quaternion input);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

MotionOdometry::MotionOdometry()
    : Node("MotionOdometry"),
      odometry_to_support_foot_(tf2::Transform::getIdentity()),
      odom_to_base_link_(tf2::Transform::getIdentity()),
      previous_odom_to_base_link_internal_(tf2::Transform::getIdentity()),
      previous_imu_orientation_in_base_link_(tf2::Quaternion::getIdentity()),
      param_listener_(get_node_parameters_interface()),
      config_(param_listener_.get_params()),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_, this),
      br_(std::make_unique<tf2_ros::TransformBroadcaster>(this)) {
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->get_parameter("base_link_frame", base_link_frame_);
  this->declare_parameter<std::string>("imu_frame", "imu_frame");
  this->get_parameter("imu_frame", imu_frame_);
  this->declare_parameter<std::string>("r_sole_frame", "r_sole");
  this->get_parameter("r_sole_frame", r_sole_frame_);
  this->declare_parameter<std::string>("r_front_left_corner_frame", "r_sole_front_left");
  this->get_parameter("r_front_left_corner_frame", r_front_left_corner_frame_);
  this->declare_parameter<std::string>("r_front_right_corner_frame", "r_sole_front_right");
  this->get_parameter("r_front_right_corner_frame", r_front_right_corner_frame_);
  this->declare_parameter<std::string>("r_back_left_corner_frame", "r_sole_back_left");
  this->get_parameter("r_back_left_corner_frame", r_back_left_corner_frame_);
  this->declare_parameter<std::string>("r_back_right_corner_frame", "r_sole_back_right");
  this->get_parameter("r_back_right_corner_frame", r_back_right_corner_frame_);
  this->declare_parameter<std::string>("l_sole_frame", "l_sole");
  this->get_parameter("l_sole_frame", l_sole_frame_);
  this->declare_parameter<std::string>("l_front_left_corner_frame", "l_sole_front_left");
  this->get_parameter("l_front_left_corner_frame", l_front_left_corner_frame_);
  this->declare_parameter<std::string>("l_front_right_corner_frame", "l_sole_front_right");
  this->get_parameter("l_front_right_corner_frame", l_front_right_corner_frame_);
  this->declare_parameter<std::string>("l_back_left_corner_frame", "l_sole_back_left");
  this->get_parameter("l_back_left_corner_frame", l_back_left_corner_frame_);
  this->declare_parameter<std::string>("l_back_right_corner_frame", "l_sole_back_right");
  this->get_parameter("l_back_right_corner_frame", l_back_right_corner_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1,
                                                              std::bind(&MotionOdometry::imuCallback, this, _1));

  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("motion_odometry", 1);
  pub_support_state_ = this->create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 1);
}

void MotionOdometry::loop() {
  // Wait for tf to be available
  bitbots_utils::wait_for_tf(this->get_logger(), this->get_clock(), &tf_buffer_,
                             {base_link_frame_, r_sole_frame_, l_sole_frame_}, base_link_frame_);

  if (!imu_data_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No IMU data received");
    return;
  }

  config_ = param_listener_.get_params();

  auto target_time = imu_data_->header.stamp;
  tf2::Quaternion imu_rotation;
  tf2::fromMsg(imu_data_->orientation, imu_rotation);

  // Detect current support foot via corner frames and publish the result
  auto [lowest_corner_l, lowest_corner_r, support_state] = detectSupportFoot(imu_rotation);
  if (lowest_corner_l.empty() or lowest_corner_r.empty()) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not detect support foot");
    return;
  }

  biped_interfaces::msg::Phase phase_msg;
  phase_msg.header.stamp = target_time;
  phase_msg.phase = support_state;
  pub_support_state_->publish(phase_msg);

  // Check if the support foot changed (left->right or right->left; double support is skipped)
  // This part adds up completed steps
  if ((support_state == biped_interfaces::msg::Phase::LEFT_STANCE &&
       previous_support_state_ == biped_interfaces::msg::Phase::RIGHT_STANCE) ||
      (support_state == biped_interfaces::msg::Phase::RIGHT_STANCE &&
       previous_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE)) {
    std::string previous_contact_corner_frame, current_contact_corner_frame, previous_support_link,
        current_support_link;

    if (previous_support_state_ == biped_interfaces::msg::Phase::LEFT_STANCE) {
      previous_support_link = l_sole_frame_;
      previous_contact_corner_frame = lowest_corner_l;
      current_support_link = r_sole_frame_;
      current_contact_corner_frame = lowest_corner_r;
    } else {
      previous_support_link = r_sole_frame_;
      previous_contact_corner_frame = lowest_corner_r;
      current_support_link = l_sole_frame_;
      current_contact_corner_frame = lowest_corner_l;
    }

    try {
      // Add the transform between previous and current support link to the odometry transform.
      geometry_msgs::msg::TransformStamped previous_to_current_support_msg =
          tf_buffer_.lookupTransform(previous_support_link, current_support_link, rclcpp::Time(0));
      tf2::Transform previous_to_current_support;
      tf2::fromMsg(previous_to_current_support_msg.transform, previous_to_current_support);

      // Correct for situations where e.g. only the tip of a foot is in contact
      previous_to_current_support =
          getFootContactModelResult(imu_rotation, previous_support_link, previous_contact_corner_frame)
              .inverse() *
          previous_to_current_support *
          getFootContactModelResult(imu_rotation, current_support_link, current_contact_corner_frame);

      // Zero out z, to prevent the robot from drifting vertically
      double x = previous_to_current_support.getOrigin().x();
      double y = previous_to_current_support.getOrigin().y();
      previous_to_current_support.setOrigin({x, y, 0});
      odometry_to_support_foot_ = odometry_to_support_foot_ * previous_to_current_support;

    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not get transform between support feet: %s", ex.what());
      return;
    }

    if (support_state != biped_interfaces::msg::Phase::DOUBLE_STANCE) {
      previous_support_state_ = support_state;
    }
  }

  // This part adds the odom during the step (single support)
  try {
    std::string current_support_link, current_contact_corner_frame;
    if (support_state == biped_interfaces::msg::Phase::LEFT_STANCE) {
      current_support_link = l_sole_frame_;
      current_contact_corner_frame = lowest_corner_l;
    } else {
      current_support_link = r_sole_frame_;
      current_contact_corner_frame = lowest_corner_r;
    }

    geometry_msgs::msg::TransformStamped current_support_to_base_msg =
        tf_buffer_.lookupTransform(current_support_link, base_link_frame_, rclcpp::Time(0));
    tf2::Transform current_support_to_base;
    tf2::fromMsg(current_support_to_base_msg.transform, current_support_to_base);

    target_time = current_support_to_base_msg.header.stamp;

    // Correct with the foot contact model, as the foot might not be flat on the ground
    current_support_to_base =
        getFootContactModelResult(imu_rotation, current_support_link, current_contact_corner_frame)
            .inverse() *
        current_support_to_base;

    tf2::Transform odom_to_base_link_internal = odometry_to_support_foot_ * current_support_to_base;

    // This is essentially the old odometry fuser
    geometry_msgs::msg::TransformStamped rotation_point_in_base_link_msg =
        tf_buffer_.lookupTransform(base_link_frame_, current_contact_corner_frame, rclcpp::Time(0));
    tf2::Transform rotation_point_in_base_link;
    tf2::fromMsg(rotation_point_in_base_link_msg.transform, rotation_point_in_base_link);

    tf2::Transform imu_mounting_offset;
    try {
      geometry_msgs::msg::TransformStamped imu_mounting_transform =
          tf_buffer_.lookupTransform(imu_frame_, base_link_frame_, rclcpp::Time(0));
      fromMsg(imu_mounting_transform.transform, imu_mounting_offset);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Not able to fuse IMU data with odometry due to a tf problem: %s", ex.what());
    }

    // Get imu orientation in base_link frame
    tf2::Quaternion imu_orientation_in_base_link = imu_rotation * imu_mounting_offset.getRotation();

    // Get how far we walked since the last time
    tf2::Transform previous_to_current =
        tf2::Transform(previous_imu_orientation_in_base_link_.inverse() * imu_orientation_in_base_link,
                       ((previous_odom_to_base_link_internal_ * rotation_point_in_base_link)
                            .inverseTimes(odom_to_base_link_internal * rotation_point_in_base_link))
                           .getOrigin());

    // Apply the walked amount to the current state
    // Go from odom to base_link, then to the rotation point and apply the movement since the last time there
    tf2::Transform odom_to_rotation_point = odom_to_base_link_ * rotation_point_in_base_link * previous_to_current;

    // Go back to the base_link frame as the odom is defined between odom and base_link
    odom_to_base_link_ = odom_to_rotation_point * rotation_point_in_base_link.inverse();

    // Just to be sure, we set the rotation to the current imu orientation
    odom_to_base_link_.setRotation(imu_orientation_in_base_link);

    // Update the previous states
    previous_imu_orientation_in_base_link_ = imu_orientation_in_base_link;
    previous_odom_to_base_link_internal_ = odom_to_base_link_internal;

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return;
  }

  geometry_msgs::msg::TransformStamped odom_to_base_link_msg = geometry_msgs::msg::TransformStamped();
  odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link_);
  odom_to_base_link_msg.header.stamp = target_time;
  odom_to_base_link_msg.header.frame_id = odom_frame_;
  odom_to_base_link_msg.child_frame_id = base_link_frame_;
  br_->sendTransform(odom_to_base_link_msg);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = target_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.pose.pose.position.x = odom_to_base_link_msg.transform.translation.x;
  odom_msg.pose.pose.position.y = odom_to_base_link_msg.transform.translation.y;
  odom_msg.pose.pose.position.z = odom_to_base_link_msg.transform.translation.z;
  odom_msg.pose.pose.orientation = odom_to_base_link_msg.transform.rotation;
  pub_odometry_->publish(odom_msg);
}

std::tuple<std::string, std::string, int> MotionOdometry::detectSupportFoot(const tf2::Quaternion& imu_rotation) {
  // Each corner frame paired with the sole it belongs to
  const std::vector<std::string> corner_r = {
      r_front_left_corner_frame_,
      r_front_right_corner_frame_,
      r_back_left_corner_frame_,
      r_back_right_corner_frame_,
  };
  const std::vector<std::string> corner_l = {
      l_front_left_corner_frame_,
      l_front_right_corner_frame_,
      l_back_left_corner_frame_,
      l_back_right_corner_frame_,
  };

  std::string lowest_corner_r = "";
  double lowest_z_r = std::numeric_limits<double>::max();
  for (const auto& corner_frame : corner_r) {
    try {
      // Look up corner position in IMU frame, then rotate into IMU world space using the IMU orientation.
      // The corner with the smallest world-Z is the contact point.
      auto transform = tf_buffer_.lookupTransform(imu_frame_, corner_frame, rclcpp::Time(0));
      tf2::Vector3 corner_in_imu(transform.transform.translation.x, transform.transform.translation.y,
                                 transform.transform.translation.z);
      tf2::Vector3 corner_in_world = tf2::quatRotate(imu_rotation, corner_in_imu);
      if (corner_in_world.z() < lowest_z_r) {
        lowest_z_r = corner_in_world.z();
        lowest_corner_r = corner_frame;
      }
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
    }
  }

  std::string lowest_corner_l = "";
  double lowest_z_l = std::numeric_limits<double>::max();
  for (const auto& corner_frame : corner_l) {
    try {
      // Look up corner position in IMU frame, then rotate into IMU world space using the IMU orientation.
      // The corner with the smallest world-Z is the contact point.
      auto transform = tf_buffer_.lookupTransform(imu_frame_, corner_frame, rclcpp::Time(0));
      tf2::Vector3 corner_in_imu(transform.transform.translation.x, transform.transform.translation.y,
                                 transform.transform.translation.z);
      tf2::Vector3 corner_in_world = tf2::quatRotate(imu_rotation, corner_in_imu);
      if (corner_in_world.z() < lowest_z_l) {
        lowest_z_l = corner_in_world.z();
        lowest_corner_l = corner_frame;
      }
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
    }
  }

  int support_state;
  if (lowest_z_l < lowest_z_r) {
    support_state = biped_interfaces::msg::Phase::LEFT_STANCE;
  } else {
    support_state = biped_interfaces::msg::Phase::RIGHT_STANCE;
  }

  return {lowest_corner_l, lowest_corner_r, support_state};
}

tf2::Transform MotionOdometry::getFootContactModelResult(const tf2::Quaternion& imu_orientation,
                                                         const std::string& sole_frame,
                                                         const std::string& contact_corner_frame) {
  // Look up both frames relative to the IMU frame at most recent available time.
  // the minor timing mismatch is acceptable, the benefit of using up-to-date values outweights it
  tf2::Transform T_world_imu, T_sole_corner, T_imu_sole;
  tf2::fromMsg(tf_buffer_.lookupTransform(sole_frame, contact_corner_frame, rclcpp::Time(0)).transform, T_sole_corner);
  tf2::fromMsg(tf_buffer_.lookupTransform(imu_frame_, sole_frame, rclcpp::Time(0)).transform, T_imu_sole);

  // Remove the yaw component of the IMU orientation
  T_world_imu.setOrigin(tf2::Vector3(0, 0, 0));
  T_world_imu.setRotation(imu_orientation);

  auto RotAtCorner = T_world_imu * T_imu_sole * T_sole_corner;
  RotAtCorner.setOrigin(tf2::Vector3(0, 0, 0));
  RotAtCorner.setRotation(dropYaw(RotAtCorner.getRotation()));

  return T_sole_corner * RotAtCorner.inverse() * T_sole_corner.inverse();
}

tf2::Quaternion MotionOdometry::dropYaw(const tf2::Quaternion input) {
  // Convert tf to eigen quaternion
  Eigen::Quaterniond eigen_quat, eigen_quat_out;
  // actually we want to do this, but since the PR does not get merged, we do a workaround
  // (see https://github.com/ros2/geometry2/pull/427)
  // tf2::convert(imu_rotation, eigen_quat);
  eigen_quat = Eigen::Quaterniond(input.w(), input.x(), input.y(), input.z());

  // Remove yaw from quaternion
  rot_conv::QuatNoEYaw(eigen_quat, eigen_quat_out);

  // Convert eigen to tf quaternion
  tf2::Quaternion tf_quat_out;

  tf2::convert(eigen_quat_out, tf_quat_out);

  return tf_quat_out;
}

void MotionOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) { imu_data_ = msg; }

}  // namespace bitbots_odometry

int main(int argc, char** argv) {
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
