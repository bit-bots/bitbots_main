#include<bitbots_convenience_frames/convenience_frames.h>

ConvenienceFramesBroadcaster::ConvenienceFramesBroadcaster() {
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  pnh.param<std::string>("r_sole_frame", r_sole_frame_, "r_sole");
  pnh.param<std::string>("l_sole_frame", l_sole_frame_, "l_sole");
  pnh.param<std::string>("r_toe_frame", r_toe_frame_, "r_toe");
  pnh.param<std::string>("l_toe_frame", l_toe_frame_, "l_toe");
  pnh.param<std::string>("approach_frame", approach_frame_, "approach_frame");
  pnh.param<std::string>("ball_approach_frame", ball_approach_frame_, "ball_approach_frame");
  pnh.param<std::string>("ball_frame", ball_frame_, "ball");
  pnh.param<std::string>("right_post_frame", right_post_frame_, "right_post");
  pnh.param<std::string>("left_post_frame", left_post_frame_, "left_post");
  pnh.param<std::string>("general_post_frame", general_post_frame_, "post_");

  got_support_foot_ = false;
  ros::Subscriber walking_support_foot_subscriber = n.subscribe("walk_support_state",
                                                                1,
                                                                &ConvenienceFramesBroadcaster::supportFootCallback,
                                                                this,
                                                                ros::TransportHints().tcpNoDelay());
  ros::Subscriber dynamic_kick_support_foot_subscriber = n.subscribe("dynamic_kick_support_state",
                                                                     1,
                                                                     &ConvenienceFramesBroadcaster::supportFootCallback,
                                                                     this,
                                                                     ros::TransportHints().tcpNoDelay());
  ros::Subscriber ball_relative_subscriber = n.subscribe("balls_relative",
                                                         1,
                                                         &ConvenienceFramesBroadcaster::ballsCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber goal_relative_subscriber = n.subscribe("goal_relative",
                                                         1,
                                                         &ConvenienceFramesBroadcaster::goalCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber goal_posts_relative_subscriber = n.subscribe("goal_posts_relative",
                                                               1,
                                                               &ConvenienceFramesBroadcaster::goalPostsCallback,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());

  ros::Rate r(200.0);
  ros::Time last_published_time;
  while (ros::ok()) {
    ros::spinOnce();
    geometry_msgs::TransformStamped tf_right, // right foot in baselink frame
        tf_left,
        tf_right_toe, // right toes baselink frame
        tf_left_toe,
        support_foot, // support foot in baselink frame
        non_support_foot,
        non_support_foot_in_support_foot_frame,
        base_footprint_in_support_foot_frame,
        front_foot; // foot that is currently in front of the other, in baselink frame

    try {
      tf_right = tfBuffer_.lookupTransform(base_link_frame_, r_sole_frame_, ros::Time::now(), ros::Duration(0.1));
      tf_left = tfBuffer_.lookupTransform(base_link_frame_, l_sole_frame_, ros::Time::now(), ros::Duration(0.1));
      tf_right_toe = tfBuffer_.lookupTransform(base_link_frame_, r_toe_frame_, ros::Time::now(), ros::Duration(0.1));
      tf_left_toe = tfBuffer_.lookupTransform(base_link_frame_, l_toe_frame_, ros::Time::now(), ros::Duration(0.1));

      // compute support foot
      if (got_support_foot_) {
        if (is_left_support) {
          support_foot = tf_left;
          non_support_foot = tf_right;
        } else {
          support_foot = tf_right;
          non_support_foot = tf_left;
        }
      } else {
        // check which foot is support foot (which foot is on the ground)
        if (tf_right.transform.translation.z < tf_left.transform.translation.z) {
          support_foot = tf_right;
          non_support_foot = tf_left;
        } else {
          support_foot = tf_left;
          non_support_foot = tf_right;
        }
      }

      // check with foot is in front
      if (tf_right.transform.translation.x < tf_left.transform.translation.x) {
        front_foot = tf_left_toe;
      } else {
        front_foot = tf_right_toe;
      }

      // get the position of the non support foot in the support frame, used for computing the barycenter
      non_support_foot_in_support_foot_frame = tfBuffer_.lookupTransform(support_foot.child_frame_id,
                                                                         non_support_foot.child_frame_id,
                                                                         support_foot.header.stamp,
                                                                         ros::Duration(0.1));

      geometry_msgs::TransformStamped support_to_base_link = tfBuffer_.lookupTransform(support_foot.header.frame_id,
                                                                                       support_foot.child_frame_id,
                                                                                       support_foot.header.stamp);

      geometry_msgs::PoseStamped approach_frame;
      // x at front foot toes
      approach_frame.pose.position.x = front_foot.transform.translation.x;
      // y between feet
      tf2::Transform center_between_foot;
      double y = non_support_foot_in_support_foot_frame.transform.translation.y / 2;
      center_between_foot.setOrigin({0.0, y, 0.0});
      center_between_foot.setRotation({0, 0, 0, 1});
      tf2::Transform support_foot_tf;
      tf2::fromMsg(support_foot.transform, support_foot_tf);
      center_between_foot = support_foot_tf * center_between_foot;
      approach_frame.pose.position.y = center_between_foot.getOrigin().y();
      // z at ground leven (support foot height)
      approach_frame.pose.position.z = support_foot.transform.translation.z;

      // roll and pitch of support foot
      double roll, pitch, yaw;
      tf2::Quaternion quat;
      fromMsg(support_foot.transform.rotation, quat);
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      // yaw of front foot
      yaw = tf2::getYaw(front_foot.transform.rotation);

      // pitch and roll from support foot, yaw from base link
      tf2::Quaternion rotation;
      rotation.setRPY(roll, pitch, yaw);
      approach_frame.pose.orientation = tf2::toMsg(rotation);

      // in simulation, the time does not always advance between loop iteration
      // in that case, we do not want to republish the transform
      ros::Time now = ros::Time::now();
      if (now != last_published_time) {
        last_published_time = now;

        // set the broadcasted transform to the position and orientation of the base footprint
        tf_.header.stamp = now;
        tf_.header.frame_id = base_link_frame_;
        tf_.child_frame_id = approach_frame_;
        tf_.transform.translation.x = approach_frame.pose.position.x;
        tf_.transform.translation.y = approach_frame.pose.position.y;
        tf_.transform.translation.z = approach_frame.pose.position.z;
        tf_.transform.rotation = approach_frame.pose.orientation;
        broadcaster_.sendTransform(tf_);

        // publish ball_approach_frame 10 cm in front of approach_frame
        tf_.header.frame_id = approach_frame_;
        tf_.child_frame_id = ball_approach_frame_;
        tf_.transform.translation.x = 0.10;
        tf_.transform.translation.y = 0;
        tf_.transform.translation.z = 0;
        tf2::Quaternion rotation_baf = tf2::Quaternion(0, 0, 0, 1);
        tf_.transform.rotation = tf2::toMsg(rotation_baf);
        broadcaster_.sendTransform(tf_);
      }
    } catch (...) {
      continue;
    }
    r.sleep();
  }
}

void ConvenienceFramesBroadcaster::supportFootCallback(const bitbots_msgs::SupportState msg) {
  got_support_foot_ = true;
  is_left_support = (msg.state == bitbots_msgs::SupportState::LEFT);
}

void ConvenienceFramesBroadcaster::ballsCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg) {
  for (humanoid_league_msgs::PoseWithCertainty ball : msg->poses) {
    publishTransform(msg->header.frame_id, ball_frame_,
                    ball.pose.pose.position.x,
                    ball.pose.pose.position.y,
                    ball.pose.pose.position.z);
  }
}

void ConvenienceFramesBroadcaster::goalCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg) {
  if (msg->poses.size() > 0)
  {
    publishTransform(msg->header.frame_id, left_post_frame_,
                    msg->poses[0].pose.pose.position.x, msg->poses[0].pose.pose.position.y, msg->poses[0].pose.pose.position.z);
  }
  if (msg->poses.size() > 2)
  {
    publishTransform(msg->header.frame_id, right_post_frame_,
                     msg->poses[1].pose.pose.position.x, msg->poses[1].pose.pose.position.y, msg->poses[1].pose.pose.position.z);
  }
}

void ConvenienceFramesBroadcaster::goalPostsCallback(const humanoid_league_msgs::PoseWithCertaintyArray::ConstPtr &msg) {
  for (long i = 0; i < msg->poses.size(); i++) {
    publishTransform(msg->header.frame_id,
                     general_post_frame_ + std::to_string(i),
                     msg->poses[i].pose.pose.position.x,
                     msg->poses[i].pose.pose.position.y,
                     msg->poses[i].pose.pose.position.z);
  }
}

void ConvenienceFramesBroadcaster::publishTransform(std::string header_frame_id,
                                                    std::string child_frame_id,
                                                    double x,
                                                    double y,
                                                    double z) {
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = std::move(header_frame_id);
  transform.child_frame_id = std::move(child_frame_id);
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.rotation.w = 1;
  broadcaster_.sendTransform(transform);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "convenience_frames");
  ConvenienceFramesBroadcaster b;
}

