#ifndef HUMANOID_LEAGUE_TEAM_COMMUNICATION_INCLUDE_HUMANOID_LEAGUE_TEAM_COMMUNICATION_TEAM_COMMUNICATION_H_
#define HUMANOID_LEAGUE_TEAM_COMMUNICATION_INCLUDE_HUMANOID_LEAGUE_TEAM_COMMUNICATION_TEAM_COMMUNICATION_H_

#include <pthread.h>
#include <vector>
#include <algorithm>
#include "humanoid_league_msgs/TeamData.h"
#include "humanoid_league_msgs/ObstacleRelativeArray.h"
#include "humanoid_league_msgs/ObstacleRelative.h"
#include "humanoid_league_msgs/RobotControlState.h"
#include "humanoid_league_msgs/Strategy.h"
#include "humanoid_league_msgs/PoseWithCertainty.h"
#include "humanoid_league_msgs/PoseWithCertaintyArray.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <google/protobuf/util/time_util.h>
#include "robocup_extension.pb.h"
#include "udp_connection.h"


/*
 * This node provides ROS connections to a mitecom object. Two threads are started, one for receiving information
 * from other robots and one for sending out information to other robots and to the ROS topics. Information about
 * the current state of the robot are fetched using subscription on multiple topics.
 */

// This struct includes the data used to describe another recognized robot.
struct ObstacleData{
  float x;
  float y;
  float belief;
  int player_number;
  robocup::humanoid::Team teamcolor;
};

class TeamCommunication {
 public:
  TeamCommunication();
  void run();
  void sendThread(const ros::TimerEvent &);

 private:
  static void* startRecvThread(void* context);
  void recvThread();
  void publishData(Message received_msg);
  void strategyCallback(humanoid_league_msgs::Strategy msg);
  void robotStateCallback(humanoid_league_msgs::RobotControlState msg);
  void positionCallback(humanoid_league_msgs::PoseWithCertainty msg);
  void ballsCallback(humanoid_league_msgs::PoseWithCertaintyArray msg);
  void obstaclesCallback(const humanoid_league_msgs::ObstacleRelativeArray &msg);

  // UDP parameters
  UdpConnection *udp_connection_;

  // ROS node parameters
  ros::NodeHandle nh_;
  ros::Timer timer_;
  double frequency_ = 0.0;

  // Transformation parameters
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener transform_listener_;

  // Subscribers, Publisher, Topics
  ros::Subscriber sub_role_;
  ros::Subscriber sub_robot_state_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_ball_;
  ros::Subscriber sub_obstacles_;

  ros::Publisher publisher_;

  std::string teamdata_topic_;
  std::string strategy_topic_;
  std::string robot_state_topic_;
  std::string position_topic_;
  std::string ball_topic_;
  std::string obstacles_topic_;

  // parameters send by the TeamComm
  // Strategy
  robocup::humanoid::Role role_ = robocup::humanoid::ROLE_IDLING;
  robocup::humanoid::Action action_ = robocup::humanoid::ACTION_UNDEFINED;
  robocup::humanoid::OffensiveSide offensive_side_ = robocup::humanoid::SIDE_LEFT;
  // State
  robocup::humanoid::State state_ = robocup::humanoid::UNKNOWN_STATE;
  // Robots pose
  float position_x_ = 0;
  float position_y_ = 0;
  float position_orientation_ = 0;
  float position_belief_ = 1;
  float position_cov_[3][3];

  // Ball
  float ball_relative_x_ = 0;
  float ball_relative_y_ = 0;
  float ball_belief_ = 1;
  float time_to_position_at_ball_ = 1000;

  // Obstacles
  std::vector<ObstacleData> obstacles_;

  // Misc (static parameters)
  int player_;
  robocup::humanoid::Team team_color_ = robocup::humanoid::UNKNOWN_TEAM;

  // auxiliary variables
  double strategy_exists_ = 0;
  double position_exists_ = 0;
  double ball_exists_ = 0;
  int lifetime_ = 0;
  double belief_threshold_ = 0;
  float avg_walking_speed_ = 0;
  int obstacles_exists_ = 0;
};

#endif
