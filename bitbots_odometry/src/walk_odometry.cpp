#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class WalkOdometry {
 public:
  WalkOdometry();
 private:
  ros::Time joint_update_time_;
  char current_support_state_;
  sensor_msgs::JointState current_joint_states_;

  void supportCallback(std_msgs::Char msg);
  void jointStateCb(const sensor_msgs::JointState &msg);

};

WalkOdometry::WalkOdometry() {
  ros::NodeHandle n("~");
  current_support_state_ = 'n';
  char previous_support_state = 'n';
  std::string current_support_link = "r_sole";
  std::string next_support_link;
  ros::Subscriber support_state_sub =
      n.subscribe("/walk_support_state", 1, &WalkOdometry::supportCallback, this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber joint_state_sub =
      n.subscribe("/joint_states", 1, &WalkOdometry::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  tf2::Transform odometry_to_support_foot = tf2::Transform();
  odometry_to_support_foot.setOrigin({0, -0.1, 0});
  odometry_to_support_foot.setRotation(tf2::Quaternion(0, 0, 0, 1));

  static tf2_ros::TransformBroadcaster br;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // wait till connection with publishers has been established
  // so we do not immediately blast something the log output
  ros::Duration(0.5).sleep();

  ros::Rate r(200.0);

  while (ros::ok()) {
    ros::spinOnce();
    // only do something, if we received a support state

    //check if first support state was received
    ros::Duration joints_delta_t = ros::Time::now() - joint_update_time_;
    if (joints_delta_t.toSec() > 0.05) {
      ROS_WARN_THROTTLE(10, "No joint states received. Will not provide odometry.");
    } else {
      // check if support state changed
      if (previous_support_state != current_support_state_) {
        // check if step finished left->double or right->double support
        if (current_support_state_ == 'd' && previous_support_state != 'n') {
          if (previous_support_state == 'l') {
            current_support_link = "l_sole";
            next_support_link = "r_sole";
          } else {
            current_support_link = "r_sole";
            next_support_link = "l_sole";
          }

          try {
            // add the transform between current and next support link to the odometry transform
            geometry_msgs::TransformStamped current_to_next_support_msg =
                tf_buffer.lookupTransform(current_support_link, next_support_link, ros::Time(0));
            tf2::Transform current_to_next_support = tf2::Transform();
            tf2::fromMsg(current_to_next_support_msg.transform, current_to_next_support);
            odometry_to_support_foot = odometry_to_support_foot * current_to_next_support;
          } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }

          // update current support link for transform from foot to base link
          current_support_link = next_support_link;
        }

        // remember the support state change
        previous_support_state = current_support_state_;
      }
      //publish transform to baselink
      try {
        geometry_msgs::TransformStamped
            current_support_to_base_msg = tf_buffer.lookupTransform(current_support_link, "base_link", ros::Time(0));
        tf2::Transform current_support_to_base;
        tf2::fromMsg(current_support_to_base_msg.transform, current_support_to_base);
        tf2::Transform odom_to_base_link = odometry_to_support_foot * current_support_to_base;
        geometry_msgs::TransformStamped odom_to_base_link_msg = geometry_msgs::TransformStamped();
        odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link);
        odom_to_base_link_msg.header.stamp = ros::Time::now();
        odom_to_base_link_msg.header.frame_id = "odom";
        odom_to_base_link_msg.child_frame_id = "base_link";
        br.sendTransform(odom_to_base_link_msg);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }
  }
}

void WalkOdometry::supportCallback(const std_msgs::Char msg) {
  current_support_state_ = msg.data;
}

void WalkOdometry::jointStateCb(const sensor_msgs::JointState &msg) {
  current_joint_states_ = msg;
  joint_update_time_ = ros::Time::now();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_walk_odometry");

  WalkOdometry o;
}

