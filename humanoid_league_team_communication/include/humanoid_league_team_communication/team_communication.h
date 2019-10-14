#ifndef TEAMCOMMUNICATION_HPP
#define TEAMCOMMUNICATION_HPP

#include <pthread.h>
#include <vector>
#include "humanoid_league_msgs/BallRelative.h"
#include "humanoid_league_msgs/TeamData.h"
#include "humanoid_league_msgs/GoalRelative.h"
#include "humanoid_league_msgs/ObstaclesRelative.h"
#include "humanoid_league_msgs/ObstacleRelative.h"
#include "humanoid_league_msgs/Position2D.h"
#include "humanoid_league_msgs/RobotControlState.h"
#include "humanoid_league_msgs/Strategy.h"
#include "humanoid_league_msgs/Model.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include "mitecom.hpp"
#include <tf/transform_datatypes.h>

/*
 * This node provides ROS connections to a mitecom object. Two threads are started, one for receiving information
 * from other robots and one for sending out information to other robots and to the ROS topics. Information about
 * the current state of the robot are fetched using subscription on multiple topics.
 */

class TeamCommunication{
 public:
  TeamCommunication();
  void run();
  void sendThread(const ros::TimerEvent&);

 private:
  static void* startRecvThread(void* context);
  void recvThread();
  void publishData(const MiTeCom::TeamRobotData& team_data);
  void strategyCallback(humanoid_league_msgs::Strategy msg);
  void robotStateCallback(humanoid_league_msgs::RobotControlState msg);
  void positionCallback(const humanoid_league_msgs::Position2D& msg);
  void ballCallback(const humanoid_league_msgs::BallRelative& msg);
  void goalCallback(const humanoid_league_msgs::GoalRelative& msg);
  void obstaclesCallback(const humanoid_league_msgs::ObstaclesRelative& msg);
  void worldCallback(const humanoid_league_msgs::Model& msg);

  int avg_walking_speed_ = 0;
  int max_kicking_distance_ = 0;
  uint8_t team_color_ = humanoid_league_msgs::ObstacleRelative::ROBOT_UNDEFINED;

  uint8_t role_ = humanoid_league_msgs::Strategy::ROLE_IDLING;
  uint8_t action_ = humanoid_league_msgs::Strategy::ACTION_UNDEFINED;
  uint8_t state_ = STATE_INACTIVE;

  uint64_t position_x_ = 0;
  uint64_t position_y_ = 0;
  uint64_t position_orientation_ = 0;
  uint64_t position_belief_ = 0;

  uint64_t ball_relative_x_ = 0;
  uint64_t ball_relative_y_ = 0;
  uint64_t ball_belief_ = 0;

  /*uint64_t oppgoal_relative_x_;
  uint64_t oppgoal_relative_y_;
  uint64_t oppgoal_belief_ = 0;*/

  using Tuple3 = std::array<uint64_t, 3>;
  std::vector<Tuple3> opponent_robots_;
  std::vector<Tuple3> team_robots_;

  uint64_t time_to_position_at_ball_ = 0;
  bool time_to_position_at_ball_set_ = false;
  uint64_t offensive_side_ = 0;
  bool offensive_side_set_ = false;

  MiTeCom::mitecom mitecom_;
  double frequency_ = 0.0;
  ros::Publisher publisher_;
  bool world_model_ = false;

  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Subscriber sub_role_;
  ros::Subscriber sub_robot_state_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_world_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_ball_;
  ros::Subscriber sub_obstacles_;

  int lifetime_ = 0;
  int ball_exists_ = 0;
  int position_exists_ = 0;
  int obstacles_exists_ = 0;
  double belief_threshold_ = 0;

  std::string teamdata_topic_;
  std::string strategy_topic_;
  std::string robot_state_topic_;
  std::string goal_topic_;
  std::string world_model_topic_;
  std::string position_topic_;
  std::string ball_topic_;
  std::string obstacles_topic_;
};

#endif
