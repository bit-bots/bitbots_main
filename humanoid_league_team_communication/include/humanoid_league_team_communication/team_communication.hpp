#ifndef TEAMCOMMUNICATION_HPP
#define TEAMCOMMUNICATION_HPP

#include <math.h>
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
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
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
    static void* start_recv_thread(void*);
    void recv_thread(void);
    void send_thread();
    void publish_data(MiTeCom::TeamRobotData);
    void strategy_callback(const humanoid_league_msgs::Strategy);
    void motion_state_callback(const humanoid_league_msgs::RobotControlState);
    void position_callback(const humanoid_league_msgs::Position2D);
    void ball_callback(const humanoid_league_msgs::BallRelative);
    void goal_callback(const humanoid_league_msgs::GoalRelative);
    void obstacles_callback(const humanoid_league_msgs::ObstaclesRelative);

    int avg_walking_speed;
    int max_kicking_distance;

    uint64_t role = humanoid_league_msgs::Strategy::ROLE_IDLING;
    uint64_t action = humanoid_league_msgs::Strategy::ACTION_UNDEFINED;
    uint64_t state = STATE_INACTIVE;

    uint64_t position_x;
    uint64_t position_y;
    uint64_t position_orientation;
    uint64_t position_belief = 0;

    uint64_t ball_relative_x;
    uint64_t ball_relative_y;
    uint64_t ball_belief = 0;

    uint64_t oppgoal_relative_x;
    uint64_t oppgoal_relative_y;
    uint64_t oppgoal_belief = 0;

    using tuple3 = std::array<uint64_t, 3>;
    std::vector<tuple3> opponent_robots;
    std::vector<tuple3> team_robots;

    uint64_t time_to_position_at_ball;
    bool time_to_position_at_ball_set = false;
    uint64_t offensive_side;
    bool offensive_side_set = false;

    MiTeCom::mitecom _mitecom;
    double frequency = 0.0;
    ros::Publisher publisher;
};

#endif