#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_odometry/odometry_parameters.hpp>
#include <bitbots_utils/utils.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

namespace bitbots_odometry {

class MotionOdometry : public rclcpp::Node {
 public:
  MotionOdometry();
  void loop();

 private:
  int previous_support_state_ = biped_interfaces::msg::Phase::LEFT_STANCE;  // Start in left support, as the robot starts standing on the left foot after the dynup
  tf2::Transform odometry_to_support_foot_;
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

  // Returns {contact_corner_frame, sole_frame} of the foot with the lowest corner in IMU world space.
  std::tuple<std::string, std::string, int> detectSupportFoot(const tf2::Quaternion& imu_rotation,
                                                        const rclcpp::Time& time);
  // Returns the corrected pose of sole_frame in the IMU frame, as if the foot were flat on the ground
  // pivoted around contact_corner_frame (which is assumed to be the fixed ground contact point).
  tf2::Transform getFootContactModelResult(const rclcpp::Time& time, const tf2::Quaternion& imu_orientation,
                                           const std::string& sole_frame,
                                           const std::string& contact_corner_frame);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

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

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 1, std::bind(&MotionOdometry::imuCallback, this, _1));

  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("motion_odometry", 1);
  pub_support_state_ = this->create_publisher<biped_interfaces::msg::Phase>("motion_support_state", 1);
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
  auto [lowest_corner_l, lowest_corner_r, support_state] = detectSupportFoot(imu_rotation, target_time);
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

    std::string previous_contact_corner_frame, current_contact_corner_frame, previous_support_link, current_support_link;

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
          tf_buffer_.lookupTransform(previous_support_link, current_support_link, target_time);
      tf2::Transform previous_to_current_support;
      tf2::fromMsg(previous_to_current_support_msg.transform, previous_to_current_support);

      // Correct for situations where e.g. only the tip of a foot is in contact
      previous_to_current_support = 
        getFootContactModelResult(
          target_time, 
          imu_rotation, 
          previous_support_link, 
          previous_contact_corner_frame
        ).inverse() 
        * previous_to_current_support 
        * getFootContactModelResult(
          target_time, 
          imu_rotation, 
          current_support_link, 
          current_contact_corner_frame
      );

      // Zero out z, to prevent the robot from drifting vertically 
      double x = previous_to_current_support.getOrigin().x();
      double y = previous_to_current_support.getOrigin().y();
      previous_to_current_support.setOrigin({x, y, 0});
      tf2::Quaternion q;
      q.setRPY(0, 0, tf2::getYaw(previous_to_current_support.getRotation()));
      previous_to_current_support.setRotation(q);
      odometry_to_support_foot_ = odometry_to_support_foot_ * previous_to_current_support;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get transform between support feet: %s", ex.what());
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

    // Correct with the foot contact model, as the foot might not be flat on the ground
    current_support_to_base = getFootContactModelResult(
      target_time, 
      imu_rotation, 
      current_support_link, 
      current_contact_corner_frame).inverse() * current_support_to_base;

    tf2::Transform odom_to_base_link = odometry_to_support_foot_ * current_support_to_base;
    geometry_msgs::msg::TransformStamped odom_to_base_link_msg = geometry_msgs::msg::TransformStamped();
    odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link);
    odom_to_base_link_msg.header.stamp = current_support_to_base_msg.header.stamp;
    odom_to_base_link_msg.header.frame_id = odom_frame_;
    odom_to_base_link_msg.child_frame_id = base_link_frame_;
    br_->sendTransform(odom_to_base_link_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_support_to_base_msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_link_frame_;
    odom_msg.pose.pose.position.x = odom_to_base_link_msg.transform.translation.x;
    odom_msg.pose.pose.position.y = odom_to_base_link_msg.transform.translation.y;
    odom_msg.pose.pose.position.z = odom_to_base_link_msg.transform.translation.z;
    odom_msg.pose.pose.orientation = odom_to_base_link_msg.transform.rotation;
    pub_odometry_->publish(odom_msg);

  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return;
  }
}

std::tuple<std::string, std::string, int> MotionOdometry::detectSupportFoot(const tf2::Quaternion& imu_rotation, const rclcpp::Time& time) {

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

tf2::Transform MotionOdometry::getFootContactModelResult(const rclcpp::Time& time,
                                                         const tf2::Quaternion& imu_orientation,
                                                         const std::string& sole_frame,
                                                         const std::string& contact_corner_frame) {
  // Look up both frames relative to the IMU frame at the given time
  tf2::Transform T_imu_corner, T_imu_sole;
  tf2::fromMsg(tf_buffer_.lookupTransform(imu_frame_, contact_corner_frame, rclcpp::Time(0)).transform, T_imu_corner);
  tf2::fromMsg(tf_buffer_.lookupTransform(imu_frame_, sole_frame, rclcpp::Time(0)).transform, T_imu_sole);

  // Rotate positions and orientation into IMU world space
  const tf2::Quaternion R_world = imu_orientation;
  const tf2::Vector3 p_corner_world = tf2::quatRotate(R_world, T_imu_corner.getOrigin());
  const tf2::Vector3 p_sole_world = tf2::quatRotate(R_world, T_imu_sole.getOrigin());
  const tf2::Quaternion R_sole_world = R_world * T_imu_sole.getRotation();

  // Flat-foot target orientation: remove roll and pitch from the sole, keep yaw
  tf2::Quaternion R_sole_flat;
  R_sole_flat.setRPY(0.0, 0.0, tf2::getYaw(R_sole_world));

  // Rotation that corrects from the current tilted orientation to the flat one
  const tf2::Quaternion R_correction = R_sole_flat * R_sole_world.inverse();

  // Pivot the sole around the fixed contact corner
  const tf2::Vector3 p_sole_corrected_world =
      p_corner_world + tf2::quatRotate(R_correction, p_sole_world - p_corner_world);

  // Express the final output as trnasform sole -> corrected sole
  tf2::Transform result;
  result.setOrigin(p_sole_corrected_world - p_sole_world);
  result.setRotation(R_correction); 
  return result;
}

void MotionOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  imu_data_ = msg;
}

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
