/*
odom -> baselink
walking (X, Y, Z, rZ)
imu (rX, rY)
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Char.h>
#include <std_msgs/Time.h>
#include <tf2/utils.h>


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

  void imuCallback(const sensor_msgs::Imu msg);
  void odomCallback(const nav_msgs::Odometry msg);
};

OdometryFuser::OdometryFuser() {
  ros::NodeHandle n("~");

  tf2::Quaternion dummy_orientation;
  dummy_orientation.setRPY(0, 0, 0);
  _odom_data.pose.pose.orientation = tf2::toMsg(dummy_orientation);
  _imu_data.orientation = tf2::toMsg(dummy_orientation);

  ros::Subscriber imu_subscriber = n.subscribe("/imu/data", 1, &OdometryFuser::imuCallback, this);
  ros::Subscriber odom_subscriber = n.subscribe("/walk_odometry", 1, &OdometryFuser::odomCallback, this);

  tf = geometry_msgs::TransformStamped();

  static tf2_ros::TransformBroadcaster br;
  ros::Duration imu_delta_t;
  float msg_rate = 10.0;
  // wait till connection with publishers has been established
  // so we do not immediately blast something the log output
  ros::Duration(0.5).sleep();

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

      // get yaw from walking odometry
      tf2::Quaternion odom_orientation;
      tf2::fromMsg(_odom_data.pose.pose.orientation, odom_orientation);
      tf2::Matrix3x3 odom_rotation_matrix(odom_orientation);
      odom_rotation_matrix.getRPY(placeholder, placeholder, walking_yaw);

      // combine orientations to new quaternion
      tf2::Quaternion new_orientation;
      new_orientation.setRPY(imu_roll, imu_pitch, walking_yaw);

      // combine it all into a tf
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = _odom_data.pose.pose.position.x;
      tf.transform.translation.y = _odom_data.pose.pose.position.y;
      tf.transform.translation.z = _odom_data.pose.pose.position.z;
      tf.transform.rotation = tf2::toMsg(new_orientation);
      br.sendTransform(tf);

    } else {
      ROS_WARN_THROTTLE(msg_rate, "No Data recived! Stop publishing...");
    }

    r.sleep();
  }
}

void OdometryFuser::imuCallback(const sensor_msgs::Imu msg) {
  _imu_data = msg;
  _imu_update_time = ros::Time::now();
}

void OdometryFuser::odomCallback(const nav_msgs::Odometry msg) {
  _odom_data = msg;
  _odom_update_time = ros::Time::now();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_odometry");

  OdometryFuser o;
}

