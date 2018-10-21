#ifndef TEAMCOMMUNICATION_HPP
#define TEAMCOMMUNICATION_HPP

#include <math.h>
#include <thread>
#include <vector>
#include "humanoid_league_msgs/BallRelative.h"
#include "humanoid_league_msgs/TeamData.h"
#include "humanoid_league_msgs/GoalRelative.h"
#include "humanoid_league_msgs/ObstaclesRelative.h"
#include "humanoid_league_msgs/ObstacleRelative.h"
#include "humanoid_league_msgs/Position2D.h"
#include "humanoid_league_msgs/RobotControlState.h"
#include "humanoid_league_msgs/Strategy.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "mitecom.hpp"

/*
 * This node provides ROS connections to a mitecom object. Two threads are started, one for receiving information
 * from other robots and one for sending out information to other robots and to the ROS topics. Information about
 * the current state of the robot are fetched using subscription on multiple topics.
 */

class TeamCommunication{
public:
    TeamCommunication();
    void run();

private:
    void recv_thread();
    void send_thread();
    void publish_data(TeamRobotData);
    void strategy_callback(const humanoid_league_msgs::Strategy);
    void motion_state_callback(const humanoid_league_msgs::RobotControlState);
    void position_callback(const humanoid_league_msgs::Position2D);
    void ball_callback(const humanoid_league_msgs::BallRelative);
    void goal_callback(const humanoid_league_msgs::GoalRelative);
    void obstacles_callback(const humanoid_league_msgs::ObstaclesRelative);

    int avg_walking_speed;
    int max_kicking_distance;

    uint8 role = Strategy::ROLE_IDLING;
    uint8 action = Strategy.ACTION_UNDEFINED;
    uint8 state = STATE_INACTIVE;

    uint8 position_x;
    uint8 position_y;
    uint8 position_orientation;
    uint8 position_belief = 0;

    uint8 ball_relative_x;
    uint8 ball_relative_y;
    uint8 ball_belief = 0;

    uint8 oppgoal_relative_x;
    uint8 oppgoal_relative_y;
    uint8 oppgoal_belief = 0;

    std::vector<unit8[]> opponent_robots = new std::vector<unit8[]>;
    std::vector<unit8[]> team_robots = new std::vector<unit8[]>;

    uint8 time_to_position_at_ball;
    std::bool time_to_position_at_ball_set = false;
    unit8 offensive_side;
    std::bool offensive_side_set = false;

    mitecom _mitecom = new mitecom;
};

#endif