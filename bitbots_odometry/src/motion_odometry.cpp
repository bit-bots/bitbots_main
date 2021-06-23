#include <bitbots_odometry/motion_odometry.h>

MotionOdometry::MotionOdometry() {
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  bool publish_walk_odom_tf;
  n.param<bool>("publish_walk_odom_tf", publish_walk_odom_tf, false);
  pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  pnh.param<std::string>("r_sole_frame", r_sole_frame_, "r_sole");
  pnh.param<std::string>("l_sole_frame", l_sole_frame_, "l_sole");
  pnh.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh.param<double>("x_forward_scaling", x_forward_scaling_, 1);
  pnh.param<double>("x_backward_scaling", x_backward_scaling_, 1);
  pnh.param<double>("y_scaling", y_scaling_, 1);
  pnh.param<double>("yaw_scaling", yaw_scaling_, 1);
  current_support_state_ = -1;
  previous_support_state_ = -1;
  std::string previous_support_link = r_sole_frame_;
  std::string current_support_link;
  ros::Subscriber walk_support_state_sub =
      n.subscribe("walk_support_state", 1, &MotionOdometry::supportCallback, this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber kick_support_state_sub =
      n.subscribe("dynamic_kick_support_state",
                  1,
                  &MotionOdometry::supportCallback,
                  this,
                  ros::TransportHints().tcpNoDelay());
  ros::Subscriber joint_state_sub =
      n.subscribe("joint_states", 1, &MotionOdometry::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  ros::Subscriber odom_subscriber = n.subscribe("walk_engine_odometry", 1, &MotionOdometry::odomCallback, this);

  ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("motion_odometry", 1);
  // set the origin to 0. will be set correctly on recieving first support state
  odometry_to_support_foot_.setOrigin({0, 0, 0});
  odometry_to_support_foot_.setRotation(tf2::Quaternion(0, 0, 0, 1));

  ros::Time foot_change_time;

  static tf2_ros::TransformBroadcaster br;
  tf2_ros::TransformListener tf_listener(tf_buffer_);
  // wait till connection with publishers has been established
  // so we do not immediately blast something into the log output
  ros::Duration(0.5).sleep();

  ros::Rate r(200.0);

  while (ros::ok()) {
    ros::spinOnce();
    if (r.sleep()) {
      //check if joint states were received, otherwise we can't provide odometry
      ros::Duration joints_delta_t = ros::Time::now() - joint_update_time_;
      if (joints_delta_t.toSec() > 0.05) {
        ROS_WARN_THROTTLE(30, "No joint states received. Will not provide odometry.");
      } else {
        // check if step finished, meaning left->right or right->left support. double support is skipped
        // the support foot change is published when the joint goals for the last movements are published.
        // it takes some time till the joints actually reach this position, this can create some offset
        // but since we skip the double support phase, we basically take the timepoint when the double support phase is
        // finished. This means both feet did not move and this should create no offset.
        if ((current_support_state_ == bitbots_msgs::SupportState::LEFT
            && previous_support_state_ == bitbots_msgs::SupportState::RIGHT) ||
            (current_support_state_ == bitbots_msgs::SupportState::RIGHT
                && previous_support_state_ == bitbots_msgs::SupportState::LEFT)) {
          foot_change_time = current_support_state_time_;
          if (previous_support_state_ == bitbots_msgs::SupportState::LEFT) {
            previous_support_link = l_sole_frame_;
            current_support_link = r_sole_frame_;
          } else {
            previous_support_link = r_sole_frame_;
            current_support_link = l_sole_frame_;
          }

          try {
            // add the transform between previous and current support link to the odometry transform.
            // we wait a bit for the transform as the joint messages are maybe a bit behind
            geometry_msgs::TransformStamped previous_to_current_support_msg =
                tf_buffer_
                    .lookupTransform(previous_support_link, current_support_link, foot_change_time, ros::Duration(0.1));
            tf2::Transform previous_to_current_support = tf2::Transform();
            tf2::fromMsg(previous_to_current_support_msg.transform, previous_to_current_support);
            // setting translation in z axis, pitch and roll to zero to stop the robot from lifting up
            // scale odometry based on parameters
            double x = previous_to_current_support.getOrigin().x();
            if (x > 0){
                x = x * x_forward_scaling_;
            }else{
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
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }

          // update current support link for transform from foot to base link
          previous_support_link = current_support_link;

          // remember the support state change but skip the double support phase
          if (current_support_state_ != bitbots_msgs::SupportState::DOUBLE) {
            previous_support_state_ = current_support_state_;
          }
        }

        //publish odometry and if wanted transform to base_link
        try {
          geometry_msgs::TransformStamped
              current_support_to_base_msg =
              tf_buffer_.lookupTransform(previous_support_link, base_link_frame_, ros::Time(0));
          tf2::Transform current_support_to_base;
          tf2::fromMsg(current_support_to_base_msg.transform, current_support_to_base);
          tf2::Transform odom_to_base_link = odometry_to_support_foot_ * current_support_to_base;
          geometry_msgs::TransformStamped odom_to_base_link_msg = geometry_msgs::TransformStamped();
          odom_to_base_link_msg.transform = tf2::toMsg(odom_to_base_link);
          odom_to_base_link_msg.header.stamp = current_support_to_base_msg.header.stamp;
          odom_to_base_link_msg.header.frame_id = odom_frame_;
          odom_to_base_link_msg.child_frame_id = base_link_frame_;
          if (publish_walk_odom_tf) {
            ROS_WARN_ONCE("Sending Tf from walk odometry directly");
            br.sendTransform(odom_to_base_link_msg);
          }

          // odometry as message
          nav_msgs::Odometry odom_msg;
          odom_msg.header.stamp = current_support_to_base_msg.header.stamp;
          odom_msg.header.frame_id = odom_frame_;
          odom_msg.child_frame_id = base_link_frame_;
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
    } else {
      sleep(0.0001);
    }
  }
}

void MotionOdometry::supportCallback(const bitbots_msgs::SupportState msg) {
  current_support_state_ = msg.state;
  current_support_state_time_ = msg.header.stamp;

  // remember if we received first support state, only remember left or right
  if (previous_support_state_ == -1 && current_support_state_ != bitbots_msgs::SupportState::DOUBLE) {
    std::string current_support_link;
    if (current_support_state_ == bitbots_msgs::SupportState::LEFT) {
      previous_support_state_ = bitbots_msgs::SupportState::RIGHT;
      current_support_link = l_sole_frame_;
    } else {
      previous_support_state_ = bitbots_msgs::SupportState::LEFT;
      current_support_link = r_sole_frame_;
    }
    // on receiving first support state we should also set the location in the world correctly
    // we assume that our baseline is on x=0 and y=0
    try{
        geometry_msgs::TransformStamped
                base_to_current_support_msg = tf_buffer_.lookupTransform(base_link_frame_, current_support_link, ros::Time(0), ros::Duration(10.0));
        odometry_to_support_foot_.setOrigin({-1 * base_to_current_support_msg.transform.translation.x,
                                            -1 * base_to_current_support_msg.transform.translation.y, 0});
    }catch (tf2::TransformException &ex){
        ROS_WARN("Could not initialize motion odometry correctly, since there were no transforms available fast enough on startup. Will initialize with 0,0,0");
    }
  }
}

void MotionOdometry::jointStateCb(const sensor_msgs::JointState &msg) {
  current_joint_states_ = msg;
  joint_update_time_ = ros::Time::now();
}

void MotionOdometry::odomCallback(nav_msgs::Odometry msg) {
  current_odom_msg_ = msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_walk_odometry");

  MotionOdometry o;
}
