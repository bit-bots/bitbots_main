/*
odom -> baselink
walking (X, Y, Z, rZ)
imu (rX, rY)
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>

// TODO Doku

class OdometryFuser {
 public:
  OdometryFuser();
 private:
  sensor_msgs::Imu _imu_data;
  nav_msgs::Odometry _odom_data;
  ros::Time _imu_update_time;
  ros::Time _odom_update_time;
  geometry_msgs::TransformStamped tf;
  char current_support_state_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void supportCallback(const std_msgs::Char::ConstPtr &msg);
  tf2::Transform getCurrentRotationPoint();

};

OdometryFuser::OdometryFuser() : tf_listener_(tf_buffer_) {
  ros::NodeHandle n("~");
  current_support_state_ = 'n';

  tf2::Quaternion dummy_orientation;
  dummy_orientation.setRPY(0, 0, 0);
  _odom_data.pose.pose.orientation = tf2::toMsg(dummy_orientation);
  _odom_data.pose.pose.position = geometry_msgs::Point();
  _imu_data.orientation = tf2::toMsg(dummy_orientation);

  ros::Subscriber imu_subscriber = n.subscribe("/imu/data", 1, &OdometryFuser::imuCallback, this);
  ros::Subscriber odom_subscriber = n.subscribe("/motion_odometry", 1, &OdometryFuser::odomCallback, this);
  ros::Subscriber walk_support_state_sub = n.subscribe("/walk_support_state", 1, &OdometryFuser::supportCallback,
                                                       this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber kick_support_state_sub = n.subscribe("/dynamic_kick_support_state", 1,
                                                       &OdometryFuser::supportCallback, this,
                                                       ros::TransportHints().tcpNoDelay());

  tf = geometry_msgs::TransformStamped();

  static tf2_ros::TransformBroadcaster br;
  ros::Duration imu_delta_t;
  // This specifies the throttle of error messages
  float msg_rate = 10.0;
  // wait till connection with publishers has been established
  // so we do not immediately blast something the log output
  ros::Duration(0.5).sleep();

  // wait for transforms from joints
  while (!tf_buffer_.canTransform("l_sole", "base_link", ros::Time(0), ros::Duration(1)) && ros::ok()) {
    ROS_WARN("Wainting for transforms from robot joints");
  }

  ros::Rate r(200.0);
  while (ros::ok()) {
    ros::spinOnce();

    imu_delta_t = ros::Time::now() - _imu_update_time;

    bool imu_active = true;
    if (imu_delta_t.toSec() > 0.5) {
      ROS_WARN_THROTTLE(msg_rate, "IMU message outdated!");
      imu_active = false;
    }

    ros::Duration odom_delta_t = ros::Time::now() - _odom_update_time;

    bool odom_active = true;
    if (odom_delta_t.toSec() > 0.5) {
      ROS_WARN_THROTTLE(msg_rate, "Odom message outdated!");
      odom_active = false;
    }

    if (imu_active || odom_active) {
      double placeholder, imu_roll, imu_pitch, walking_yaw;

      // get roll an pitch from imu
      tf2::Quaternion imu_orientation;
      tf2::fromMsg(_imu_data.orientation, imu_orientation);
      tf2::Matrix3x3 imu_rotation_matrix(imu_orientation);
      imu_rotation_matrix.getRPY(imu_roll, imu_pitch, placeholder);

      // get motion_odom transform
      tf2::Transform motion_odometry;
      tf2::fromMsg(_odom_data.pose.pose, motion_odometry);

      // combine orientations to new quaternion if IMU is active, use purely odom otherwise
      tf2::Transform fused_odometry;
      if (imu_active) {
        // compute the point of rotation (in base_link frame)
        tf2::Transform rotation_point_in_base = getCurrentRotationPoint();
        // publish rotation point as debug
        geometry_msgs::TransformStamped rotation_point_msg;
        rotation_point_msg.header.stamp = ros::Time::now();
        rotation_point_msg.header.frame_id = "base_link";
        rotation_point_msg.child_frame_id = "rotation";
        geometry_msgs::Transform rotation_point_transform_msg;
        rotation_point_transform_msg = tf2::toMsg(rotation_point_in_base);
        rotation_point_msg.transform = rotation_point_transform_msg;
        br.sendTransform(rotation_point_msg);
        // get base_link in rotation point frame
        tf2::Transform base_link_in_rotation_point = rotation_point_in_base.inverse();
        // create transform that rotates around IMU roll,pitch and walk yaw
        tf2::Quaternion rotation_quat;
        rotation_quat.setRPY(imu_roll, imu_pitch, 0);
        tf2::Transform rotation;
        rotation.setRotation(rotation_quat);
        rotation.setOrigin({0, 0, 0});
        
        // get only translation and yaw from motion odometry
        tf2::Quaternion odom_orientation = motion_odometry.getRotation();
        tf2::Matrix3x3 odom_rotation_matrix(odom_orientation);
        odom_rotation_matrix.getRPY(placeholder, placeholder, walking_yaw);
        tf2::Transform motion_odometry_yaw;
        tf2::Quaternion odom_orientation_yaw;
        odom_orientation_yaw.setRPY(0, 0, walking_yaw);
        motion_odometry_yaw.setRotation(odom_orientation_yaw);
        motion_odometry_yaw.setOrigin(motion_odometry.getOrigin());

        // transformation chain to get correctly rotated odom frame
        // go to the rotation point in the odom frame. rotate the transform to the base link at this point
        fused_odometry = motion_odometry_yaw * rotation_point_in_base * rotation * base_link_in_rotation_point;
      } else {
        fused_odometry = motion_odometry;
      }

      // combine it all into a tf
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      geometry_msgs::Transform fused_odom_msg;
      fused_odom_msg = toMsg(fused_odometry);
      tf.transform = fused_odom_msg;
      br.sendTransform(tf);

    } else {
      ROS_WARN_THROTTLE(msg_rate, "No Data received! Stop publishing...");
    }

    r.sleep();
  }
}

tf2::Transform OdometryFuser::getCurrentRotationPoint() {
  geometry_msgs::TransformStamped rotation_point;
  tf2::Transform rotation_point_tf;

  // if center of pressure is available, it is the point of rotation
  try {
    rotation_point = tf_buffer_.lookupTransform("base_link", "cop", ros::Time(0));
    fromMsg(rotation_point.transform, rotation_point_tf);
  } catch (tf2::TransformException ex) {
    // otherwise point of rotation is current support foot sole or center point of the soles if double support
    if (current_support_state_ == 'r' || current_support_state_ == 'l') {
      try {
        rotation_point = tf_buffer_.lookupTransform("base_link", std::string("") + current_support_state_ + "_sole",
                                                    ros::Time(0));
        fromMsg(rotation_point.transform, rotation_point_tf);
      } catch (tf2::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
    } else if (current_support_state_ == 'd' || current_support_state_ == 'n') {
      // use point between soles if double support or unknown support
      geometry_msgs::TransformStamped base_to_l_sole;
      base_to_l_sole = tf_buffer_.lookupTransform("base_link", "l_sole", ros::Time(0));
      geometry_msgs::TransformStamped l_to_r_sole;
      l_to_r_sole = tf_buffer_.lookupTransform("l_sole", "r_sole", ros::Time(0));
      tf2::Transform base_to_l_sole_tf;
      tf2::fromMsg(base_to_l_sole.transform, base_to_l_sole_tf);
      tf2::Transform l_to_r_sole_tf;
      tf2::fromMsg(l_to_r_sole.transform, l_to_r_sole_tf);

      // we only want to have the half transform to get the point between the feet
      tf2::Transform l_to_center_tf;
      l_to_center_tf
          .setOrigin({l_to_r_sole_tf.getOrigin().x() /2, l_to_r_sole_tf.getOrigin().y() /2, l_to_r_sole_tf.getOrigin().z() /2});
      tf2::Matrix3x3 rotation_matrix(l_to_r_sole_tf.getRotation());
      double roll, pitch, yaw;
      rotation_matrix.getRPY(roll, pitch, yaw);
      tf2::Quaternion quat;
      quat.setRPY(roll / 2, pitch / 2, yaw / 2);
      quat.normalize();
      l_to_center_tf.setRotation(quat);

      rotation_point_tf = base_to_l_sole_tf * l_to_center_tf;
      rotation_point_tf.setRotation(rotation_point_tf.getRotation().normalize());
    } else {
      ROS_ERROR_THROTTLE(2, "cop not available and unknown support state %c", current_support_state_);
    }
  }
  return rotation_point_tf;
}

void OdometryFuser::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  _imu_data = *msg;
  _imu_update_time = ros::Time::now();
}

void OdometryFuser::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  _odom_data = *msg;
  _odom_update_time = ros::Time::now();
}

void OdometryFuser::supportCallback(const std_msgs::Char::ConstPtr &msg) {
  current_support_state_ = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_odometry");

  OdometryFuser o;
}

