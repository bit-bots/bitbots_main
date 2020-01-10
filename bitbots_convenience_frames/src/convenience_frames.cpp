#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <humanoid_league_msgs/BallRelative.h>
#include <humanoid_league_msgs/GoalRelative.h>
#include <humanoid_league_msgs/GoalPartsRelative.h>

#include <utility>

class ConvenienceFramesBroadcaster {
 public:
  ConvenienceFramesBroadcaster();
 private:
  tf2_ros::TransformBroadcaster broadcaster_{tf2_ros::TransformBroadcaster()};
  geometry_msgs::TransformStamped tf_{geometry_msgs::TransformStamped()};
  tf2_ros::Buffer tfBuffer_{ros::Duration(1.0)};
  tf2_ros::TransformListener tfListener_{tfBuffer_};
  bool is_left_support{false};
  bool got_support_foot_{false};
  void publishTransform(std::string header_frame_id, std::string child_frame_id, double x, double y, double z);
  void supportFootCallback(const std_msgs::Char::ConstPtr &msg);
  void ballCallback(const humanoid_league_msgs::BallRelative::ConstPtr &msg);
  void goalCallback(const humanoid_league_msgs::GoalRelative::ConstPtr &msg);
  void goalPartsCallback(const humanoid_league_msgs::GoalPartsRelative::ConstPtr &msg);
};

ConvenienceFramesBroadcaster::ConvenienceFramesBroadcaster() {
  ros::NodeHandle n("~");
  got_support_foot_ = false;
  ros::Subscriber walking_support_foot_subscriber = n.subscribe("/walk_support_state",
                                                                1,
                                                                &ConvenienceFramesBroadcaster::supportFootCallback,
                                                                this,
                                                                ros::TransportHints().tcpNoDelay());
  ros::Subscriber dynamic_kick_support_foot_subscriber = n.subscribe("/dynamic_kick_support_state",
                                                                     1,
                                                                     &ConvenienceFramesBroadcaster::supportFootCallback,
                                                                     this,
                                                                     ros::TransportHints().tcpNoDelay());
  ros::Subscriber ball_relative_subscriber = n.subscribe("/ball_relative",
                                                         1,
                                                         &ConvenienceFramesBroadcaster::ballCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber goal_relative_subscriber = n.subscribe("/goal_relative",
                                                         1,
                                                         &ConvenienceFramesBroadcaster::goalCallback,
                                                         this,
                                                         ros::TransportHints().tcpNoDelay());
  ros::Subscriber goal_parts_relative_subscriber = n.subscribe("/goal_parts_relative",
                                                               1,
                                                               &ConvenienceFramesBroadcaster::goalPartsCallback,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());

  ros::Rate r(200.0);
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
      tf_right = tfBuffer_.lookupTransform("base_link", "r_sole", ros::Time::now(), ros::Duration(0.1));
      tf_left = tfBuffer_.lookupTransform("base_link", "l_sole", ros::Time::now(), ros::Duration(0.1));
      tf_right_toe = tfBuffer_.lookupTransform("base_link", "r_toe", ros::Time::now(), ros::Duration(0.1));
      tf_left_toe = tfBuffer_.lookupTransform("base_link", "l_toe", ros::Time::now(), ros::Duration(0.1));

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

      // set the broadcasted transform to the position and orientation of the base footprint
      tf_.header.stamp = ros::Time::now();
      tf_.header.frame_id = "base_link";
      tf_.child_frame_id = "approach_frame";
      tf_.transform.translation.x = approach_frame.pose.position.x;
      tf_.transform.translation.y = approach_frame.pose.position.y;
      tf_.transform.translation.z = approach_frame.pose.position.z;
      tf_.transform.rotation = approach_frame.pose.orientation;
      broadcaster_.sendTransform(tf_);
    } catch (...) {
      continue;
    }
    r.sleep();
  }
}

void ConvenienceFramesBroadcaster::supportFootCallback(const std_msgs::Char::ConstPtr &msg) {
  got_support_foot_ = true;
  is_left_support = (msg->data == 'l');
}

void ConvenienceFramesBroadcaster::ballCallback(const humanoid_league_msgs::BallRelative::ConstPtr &msg) {
  publishTransform(msg->header.frame_id, "ball",
                   msg->ball_relative.x, msg->ball_relative.y, msg->ball_relative.z);
}

void ConvenienceFramesBroadcaster::goalCallback(const humanoid_league_msgs::GoalRelative::ConstPtr &msg) {
  publishTransform(msg->header.frame_id, "left_post",
                   msg->left_post.x, msg->left_post.y, msg->left_post.z);
  publishTransform(msg->header.frame_id, "right_post",
                   msg->right_post.x, msg->right_post.y, msg->right_post.z);
}

void ConvenienceFramesBroadcaster::goalPartsCallback(const humanoid_league_msgs::GoalPartsRelative::ConstPtr &msg) {
  for (long i = 0; i < msg->posts.size(); i++) {
    publishTransform(msg->header.frame_id,
                     "post_" + std::to_string(i),
                     msg->posts[i].foot_point.x,
                     msg->posts[i].foot_point.y,
                     msg->posts[i].foot_point.z);
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

