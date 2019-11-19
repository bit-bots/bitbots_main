#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>


class WalkOdometry {
 public:
  WalkOdometry();
 private:
  ros::Time joint_update_time_;
  char current_support_state_;
  char previous_support_state_;
  sensor_msgs::JointState current_joint_states_;
  nav_msgs::Odometry current_odom_msg_;

  void supportCallback(std_msgs::Char msg);
  void jointStateCb(const sensor_msgs::JointState &msg);
  void odomCallback(nav_msgs::Odometry msg);
};

WalkOdometry::WalkOdometry() {
  // todo think about if this node should be integrated into the walk node
  ros::NodeHandle n("~");
  bool publish_walk_odom_tf;
  n.param<bool>("/publish_walk_odom_tf", publish_walk_odom_tf, false);
  current_support_state_ = 'n';
  previous_support_state_ = 'n';
  std::string current_support_link = "r_sole";
  std::string next_support_link;
  ros::Subscriber support_state_sub =
      n.subscribe("/walk_support_state", 1, &WalkOdometry::supportCallback, this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber joint_state_sub =
      n.subscribe("/joint_states", 1, &WalkOdometry::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber odom_subscriber = n.subscribe("/walk_odometry_engine", 1, &WalkOdometry::odomCallback, this);

  ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("/walk_odometry", 1);
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

    //check if joint states were received, otherwise we can't provide odometry
    ros::Duration joints_delta_t = ros::Time::now() - joint_update_time_;
    if (joints_delta_t.toSec() > 0.05) {
      ROS_WARN_THROTTLE(10, "No joint states received. Will not provide odometry.");
    } else {
      // check if step finished, meaning left->right or right->left support. double support is skipped
      if ((current_support_state_ == 'l' && previous_support_state_ == 'r') ||
          (current_support_state_ == 'r' && previous_support_state_ == 'l')) {
        if (previous_support_state_ == 'l') {
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
          // setting translation in z axis, pitch and roll to zero to stop the robot from lifting up
          current_to_next_support.setOrigin({
                                                current_to_next_support.getOrigin().x(),
                                                current_to_next_support.getOrigin().y(),
                                                0});
          tf2::Quaternion q;
          q.setRPY(0, 0, tf2::getYaw(current_to_next_support.getRotation()));
          current_to_next_support.setRotation(q);
          odometry_to_support_foot = odometry_to_support_foot * current_to_next_support;
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // update current support link for transform from foot to base link
        current_support_link = next_support_link;

        // remember the support state change
        previous_support_state_ = current_support_state_;
      }

      //publish odometry and if wanted transform to base_link
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
        if (publish_walk_odom_tf) {
          ROS_WARN_ONCE("Sending Tf from walk odometry directly");
          br.sendTransform(odom_to_base_link_msg);
        }

        // odometry as message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = odom_to_base_link_msg.transform.translation.x;
        odom_msg.pose.pose.position.y = odom_to_base_link_msg.transform.translation.y;
        odom_msg.pose.pose.position.z = odom_to_base_link_msg.transform.translation.z;
        odom_msg.pose.pose.orientation = odom_to_base_link_msg.transform.rotation;
        odom_msg.twist = current_odom_msg_.twist;
        pub_odometry.publish(odom_msg);

      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.1).sleep();
        continue;
      }
    }
    r.sleep();
  }
}

void WalkOdometry::supportCallback(const std_msgs::Char msg) {
  current_support_state_ = msg.data;

  // remember if we recieved first support state, only remember left or right
  if (previous_support_state_ == 'n' && current_support_state_ != 'd') {
    previous_support_state_ = current_support_state_;
  }
}

void WalkOdometry::jointStateCb(const sensor_msgs::JointState &msg) {
  current_joint_states_ = msg;
  joint_update_time_ = ros::Time::now();
}

void WalkOdometry::odomCallback(nav_msgs::Odometry msg){
  current_odom_msg_ = msg;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_walk_odometry");

  WalkOdometry o;
}
