/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef QUINTICWALKNODE_HPP
#define QUINTICWALKNODE_HPP

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <chrono>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <moveit_msgs/RobotState.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_quintic_walk/WalkingDebug.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_msgs/FootPressure.h>

#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h> 

#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
#include "bitbots_ik/AnalyticIKSolver.hpp"
#include "bitbots_ik/BioIKSolver.hpp"
#include "bitbots_quintic_walk/WalkEngine.hpp"
#include <std_msgs/Bool.h>
#include <unistd.h>


class QuinticWalkingNode {
public:
    QuinticWalkingNode();
    void run();
    void reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);
    void initilizeEngine();

private:
    void publishControllerCommands(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishDebug(tf::Transform& trunk_to_support_foot, tf::Transform& trunk_to_flying_foot);
    void publishMarker(std::string name_space, std::string frame, geometry_msgs::Pose pose, float r, float g, float b, float a);
    void publishMarkers();
    void publishTrajectoryDebug();
    void publishOdometry();

    void cmdVelCb(const geometry_msgs::Twist msg);
    void imuCb(const sensor_msgs::Imu msg);
    void pressureCb(const bitbots_msgs::FootPressure msg);
    void robStateCb(const humanoid_league_msgs::RobotControlState msg);
    void jointStateCb(const sensor_msgs::JointState msg);
    void kickCb(const std_msgs::BoolConstPtr msg);

    void calculateJointGoals();
    double getTimeDelta();

    bool _debugActive;
    bool _simulation_active;

    bool _first_run;

    double _engineFrequency;

    bool _phaseResetActive;
    double _groundMinPressure;
    double _copStopActive;
    double _ioPressureThreshold;
    double _fbPressureThreshold;

    bool _imuActive;
    double _imu_pitch_threshold;
    double _imu_roll_threshold;

    int _odomPubFactor;
    std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
    double _last_ros_update_time;

    int _robotState;
    int _marker_id;

    bitbots_quintic_walk::WalkingParameter _params;

    Eigen::Vector3d _trunkPos;
    Eigen::Vector3d _trunkAxis;
    Eigen::Vector3d _footPos;
    Eigen::Vector3d _footAxis;
    bool _isLeftSupport;

    Eigen::Vector3d _currentOrders;
    Eigen::Vector3d _max_step;
    bitbots_quintic_walk::QuinticWalk _walkEngine;

    bitbots_msgs::JointCommand _command_msg;
    nav_msgs::Odometry _odom_msg;
    geometry_msgs::TransformStamped _odom_trans;

    ros::NodeHandle _nh;

    ros::Publisher _pubControllerCommand;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubSupport;
    tf::TransformBroadcaster _odom_broadcaster;
    ros::Publisher _pubDebug;
    ros::Publisher _pubDebugMarker;

    ros::Subscriber _subCmdVel;
    ros::Subscriber _subRobState;
    ros::Subscriber _subJointStates;
    ros::Subscriber _subKick;
    ros::Subscriber _subImu;
    ros::Subscriber _subPressure;

    // MoveIt!
    robot_model_loader::RobotModelLoader _robot_model_loader;
    robot_model::RobotModelPtr _kinematic_model;
    robot_state::RobotStatePtr _goal_state;
    robot_state::RobotStatePtr _current_state;
    const robot_state::JointModelGroup *_all_joints_group;
    const robot_state::JointModelGroup *_legs_joints_group;
    const robot_state::JointModelGroup *_lleg_joints_group;
    const robot_state::JointModelGroup *_rleg_joints_group;
    
    // IK solver
    bitbots_ik::BioIKSolver _bioIK_solver;

};

#endif