/*
This node fuses the odometry data from the walk engine with the IMU data.
It rotates the robot around it's foot contact point.

odom -> baselink
walking (X, Y, Z, rZ)
imu (rX, rY)
*/

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rot_conv/rot_conv.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_utils/utils.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using bitbots_utils::wait_for_tf;
using std::placeholders::_1;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, nav_msgs::msg::Odometry> SyncPolicy;

class OdometryFuser : public rclcpp::Node {
 public:
  OdometryFuser()
      : Node("OdometryFuser"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this)),
        support_state_cache_(100),
        imu_sub_(this, "imu/data"),
        motion_odom_sub_(this, "motion_odometry"),
        br_(std::make_unique<tf2_ros::TransformBroadcaster>(this)),
        sync_(message_filters::Synchronizer<SyncPolicy>(SyncPolicy(50), imu_sub_, motion_odom_sub_)) {
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->get_parameter("base_link_frame", base_link_frame_);
    this->declare_parameter<std::string>("r_sole_frame", "r_sole");
    this->get_parameter("r_sole_frame", r_sole_frame_);
    this->declare_parameter<std::string>("l_sole_frame", "l_sole");
    this->get_parameter("l_sole_frame", l_sole_frame_);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);
    this->declare_parameter<std::string>("rotation_frame", "rotation");
    this->get_parameter("rotation_frame", rotation_frame_);

    walk_support_state_sub_ = this->create_subscription<biped_interfaces::msg::Phase>(
        "walk_support_state", 1, std::bind(&OdometryFuser::supportCallback, this, _1));
    kick_support_state_sub_ = this->create_subscription<biped_interfaces::msg::Phase>(
        "dynamic_kick_support_state", 1, std::bind(&OdometryFuser::supportCallback, this, _1));

    sync_.registerCallback(&OdometryFuser::imuCallback, this);
    start_time_ = this->now();
    fused_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void loop() {
    bitbots_utils::wait_for_tf(this->get_logger(), this->get_clock(), this->tf_buffer_.get(),
                               {base_link_frame_, r_sole_frame_, l_sole_frame_}, base_link_frame_);

    // get motion_odom transform
    tf2::Transform motion_odometry;
    tf2::fromMsg(odom_data_.pose.pose, motion_odometry);

    // combine orientations to new quaternion if IMU is active, use purely odom otherwise
    tf2::Transform fused_odometry{motion_odometry};

    // Check if the imu is active
    if (imu_data_received_) {
      // Check if fused_time_ is recent enough
      // Otherwise it might happen that we try to process a time that is too old
      // it it is older than the tf buffer content, we will wait for the tf to be updated.
      // This wait clogs the executor, leading to a deadlock. It locks because with the clogged executor,
      // the fused_time_ is not updated, because the callback updating it is not executed
      if (this->now() - fused_time_ > rclcpp::Duration::from_seconds(0.1)) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 2 * 1000,
                                       "Fused time is too old, this should only happen if we get no new data");
        return;
      }

      // get roll an pitch from imu
      tf2::Quaternion imu_orientation;
      tf2::fromMsg(imu_data_.orientation, imu_orientation);

      // compute the point of rotation (in base_link frame)
      tf2::Transform rotation_point_in_base = getCurrentRotationPoint();
      // get base_link in rotation point frame
      tf2::Transform base_link_in_rotation_point = rotation_point_in_base.inverse();

      // get only translation and yaw from motion odometry
      tf2::Quaternion odom_orientation_yaw = getCurrentMotionOdomYaw(motion_odometry.getRotation());
      tf2::Transform motion_odometry_yaw;
      motion_odometry_yaw.setRotation(odom_orientation_yaw);
      motion_odometry_yaw.setOrigin(motion_odometry.getOrigin());

      // Get the rotation offset between the IMU and the baselink
      tf2::Transform imu_mounting_offset;
      try {
        geometry_msgs::msg::TransformStamped imu_mounting_transform =
            tf_buffer_->lookupTransform(imu_data_.header.frame_id, base_link_frame_, fused_time_);
        fromMsg(imu_mounting_transform.transform, imu_mounting_offset);
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Not able to fuse IMU data with odometry due to a tf problem: %s", ex.what());
      }

      // get imu transform without yaw
      tf2::Quaternion imu_orientation_without_yaw_component =
          getCurrentImuRotationWithoutYaw(imu_orientation * imu_mounting_offset.getRotation());
      tf2::Transform imu_without_yaw_component;
      imu_without_yaw_component.setRotation(imu_orientation_without_yaw_component);
      imu_without_yaw_component.setOrigin({0, 0, 0});

      // transformation chain to get correctly rotated odom frame
      // go to the rotation point in the odom frame. rotate the transform to the base link at this point
      fused_odometry =
          motion_odometry_yaw * rotation_point_in_base * imu_without_yaw_component * base_link_in_rotation_point;
    } else {
      fused_odometry = motion_odometry;
    }

    // combine it all into a tf
    tf_.header.stamp = fused_time_;
    tf_.header.frame_id = odom_frame_;
    tf_.child_frame_id = base_link_frame_;
    geometry_msgs::msg::Transform fused_odom_msg;
    fused_odom_msg = toMsg(fused_odometry);
    tf_.transform = fused_odom_msg;
    br_->sendTransform(tf_);
  }

  void supportCallback(const biped_interfaces::msg::Phase::SharedPtr msg) { support_state_cache_.add(msg); }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr &imu_msg,
                   const nav_msgs::msg::Odometry::SharedPtr &motion_odom_msg) {
    imu_data_ = *imu_msg;
    odom_data_ = *motion_odom_msg;
    // Use the time of the imu as a baseline to do transforms and stuff because it is more timecritical than the walking
    // odometry. The walking odom stamp is also close to this timestamp due to the Synchronizer policy.
    fused_time_ = imu_data_.header.stamp;
    imu_data_received_ = true;
  }

 private:
  sensor_msgs::msg::Imu imu_data_;
  nav_msgs::msg::Odometry odom_data_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time fused_time_;
  std::string base_link_frame_, r_sole_frame_, l_sole_frame_, odom_frame_, rotation_frame_, imu_frame_;
  bool imu_data_received_ = false;

  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr walk_support_state_sub_;
  rclcpp::Subscription<biped_interfaces::msg::Phase>::SharedPtr kick_support_state_sub_;

  message_filters::Cache<biped_interfaces::msg::Phase> support_state_cache_;

  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> motion_odom_sub_;
  geometry_msgs::msg::TransformStamped tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Time start_time_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  tf2::Quaternion getCurrentMotionOdomYaw(tf2::Quaternion motion_odom_rotation) {
    // Convert tf to eigen quaternion
    Eigen::Quaterniond eigen_quat, out;
    // actually we want to do this, but since the PR does not get merged, we do a workaround
    // (see https://github.com/ros2/geometry2/pull/427)
    // tf2::convert(motion_odom_rotation, eigen_quat);
    eigen_quat = Eigen::Quaterniond(motion_odom_rotation.w(), motion_odom_rotation.x(), motion_odom_rotation.y(),
                                    motion_odom_rotation.z());

    // Extract yaw rotation
    double yaw = rot_conv::EYawOfQuat(eigen_quat);

    tf2::Quaternion odom_orientation_yaw;
    odom_orientation_yaw.setRPY(0, 0, yaw);

    return odom_orientation_yaw;
  }

  tf2::Quaternion getCurrentImuRotationWithoutYaw(tf2::Quaternion imu_rotation) {
    // Calculate robot orientation vector
    tf2::Vector3 robot_vector = tf2::Vector3(0, 0, 1);
    tf2::Vector3 robot_vector_rotated =
        robot_vector.rotate(imu_rotation.getAxis(), imu_rotation.getAngle()).normalize();

    // Check if the robots orientation is near the yaw singularity
    if (robot_vector_rotated.z() < 0.2) {
      // Use ony a IMU offset during the singularity
      return imu_rotation;
    }

    // Convert tf to eigen quaternion
    Eigen::Quaterniond eigen_quat, eigen_quat_out;
    // actually we want to do this, but since the PR does not get merged, we do a workaround
    // (see https://github.com/ros2/geometry2/pull/427)
    // tf2::convert(imu_rotation, eigen_quat);
    eigen_quat = Eigen::Quaterniond(imu_rotation.w(), imu_rotation.x(), imu_rotation.y(), imu_rotation.z());

    // Remove yaw from quaternion
    rot_conv::QuatNoEYaw(eigen_quat, eigen_quat_out);

    // Convert eigen to tf quaternion
    tf2::Quaternion tf_quat_out;

    tf2::convert(eigen_quat_out, tf_quat_out);

    return tf_quat_out;
  }

  tf2::Transform getCurrentRotationPoint() {
    geometry_msgs::msg::TransformStamped rotation_point;
    tf2::Transform rotation_point_tf;

    char current_support_state = biped_interfaces::msg::Phase::DOUBLE_STANCE;

    biped_interfaces::msg::Phase::ConstSharedPtr current_support_state_msg =
        support_state_cache_.getElemBeforeTime(fused_time_);

    if (current_support_state_msg) {
      current_support_state = current_support_state_msg->phase;
    }
    // Wait for the forward kinematics of both legs (simplified by transforming from one to the other) to be avalible
    // for the current fusing operation
    tf_buffer_->canTransform(r_sole_frame_, l_sole_frame_, fused_time_, rclcpp::Duration::from_nanoseconds(0.1 * 1e9));
    // otherwise point of rotation is current support foot sole or center point of the soles if double support
    if (current_support_state == biped_interfaces::msg::Phase::RIGHT_STANCE ||
        current_support_state == biped_interfaces::msg::Phase::LEFT_STANCE) {
      try {
        std::string support_frame;
        if (current_support_state == biped_interfaces::msg::Phase::RIGHT_STANCE)
          support_frame = r_sole_frame_;
        else
          support_frame = l_sole_frame_;
        rotation_point = tf_buffer_->lookupTransform(base_link_frame_, support_frame, fused_time_);
        fromMsg(rotation_point.transform, rotation_point_tf);
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      }
    } else if (current_support_state == biped_interfaces::msg::Phase::DOUBLE_STANCE) {
      try {
        // use point between soles if double support or unknown support
        geometry_msgs::msg::TransformStamped base_to_l_sole;
        base_to_l_sole = tf_buffer_->lookupTransform(base_link_frame_, l_sole_frame_, fused_time_);
        geometry_msgs::msg::TransformStamped l_to_r_sole;
        l_to_r_sole = tf_buffer_->lookupTransform(l_sole_frame_, r_sole_frame_, fused_time_);
        tf2::Transform base_to_l_sole_tf;
        tf2::fromMsg(base_to_l_sole.transform, base_to_l_sole_tf);
        tf2::Transform l_to_r_sole_tf;
        tf2::fromMsg(l_to_r_sole.transform, l_to_r_sole_tf);

        // we only want to have the half transform to get the point between the feet
        tf2::Transform l_to_center_tf;
        l_to_center_tf.setOrigin({l_to_r_sole_tf.getOrigin().x() / 2, l_to_r_sole_tf.getOrigin().y() / 2,
                                  l_to_r_sole_tf.getOrigin().z() / 2});

        // Set to zero rotation, because the rotation measurement is done by the imu
        tf2::Quaternion zero_rotation;
        zero_rotation.setRPY(0, 0, 0);
        l_to_center_tf.setRotation(zero_rotation);

        rotation_point_tf = base_to_l_sole_tf * l_to_center_tf;
        rotation_point_tf.setRotation(zero_rotation);
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      }
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2, "cop not available and unknown support state %c",
                            current_support_state);
    }
    return rotation_point_tf;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryFuser>();
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 100.0);
  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });

  exec.spin();
  rclcpp::shutdown();
}
