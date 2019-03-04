/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_quintic_walk/WalkingDebug.h>
#include <bitbots_msgs/JointCommand.h>

#include <dynamic_reconfigure/server.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h> 

#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
#include "bitbots_ik/AnalyticIKSolver.hpp"
#include "bitbots_ik/BioIKSolver.hpp"
#include "bitbots_quintic_walk/WalkEngine.hpp"
#include "bitbots_quintic_walk/gravity_compensator.h"
#include <std_msgs/Bool.h>  



class QuinticWalkingNode {
public:
    QuinticWalkingNode();
    void run();
    void reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

private:
    void publishModelJointStates(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishControllerCommands(std::vector <std::string> joint_names, std::vector<double> positions);
    void publishDebug(tf2::Transform& trunk_to_support_foot, tf2::Transform& trunk_to_flying_foot);
    void publishMarkers();
    void publishTrajectoryDebug();
    void publishOdometry();
    void cmdVelCb(const geometry_msgs::Twist msg);
    void imuCb(const sensor_msgs::Imu msg);
    void robStateCb(const humanoid_league_msgs::RobotControlState msg);
    void jointStateCb(const sensor_msgs::JointState msg);
    void kickCb(const std_msgs::BoolConstPtr msg);
    void walkingReset();
    void calculateWalking();
    void compensateGravity();

    bool _debugActive;
    bool _pub_model_joint_states;
    bool _imuActive;
    bool _walkActive;
    bool _simulation_active;

    bool _stopRequest;
    double _engineFrequency;

    double _imu_pitch_threshold;
    double _imu_roll_threshold;
    bool _imu_stop;

    double _vel;
    double _acc;
    double _pwm;
    int _odomPubFactor;
    std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
    double _last_ros_update_time;
    bool _just_started;

    int _robotState;
    int _marker_id;

    bitbots_quintic_walk::WalkingParameter _params;
    Eigen::Vector3d _stepOdom;
    tf2::Transform _supportFootOdom;

    std::string _ik_type;
    std::string _robot_type;

    Eigen::Vector3d _trunkPos;
    Eigen::Vector3d _trunkAxis;
    Eigen::Vector3d _footPos;
    Eigen::Vector3d _footAxis;
    bool _isLeftSupport;
    bool _wasLeftSupport;
    
    bool _compensate_gravity;
    double _gravityP;
    double _balance_left_right;
    bitbots_quintic_walk::GravityCompensator _gravity_compensator;

    Eigen::Vector3d _orders;
    Eigen::Vector3d _max_step;
    bitbots_quintic_walk::QuinticWalk _walkEngine;

    sensor_msgs::JointState _joint_state_msg;
    bitbots_msgs::JointCommand _command_msg;
    nav_msgs::Odometry _odom_msg;
    geometry_msgs::TransformStamped _odom_trans;
    trajectory_msgs::JointTrajectory _joint_traj_msg;
    moveit_msgs::DisplayTrajectory _traj_display_msg;

    ros::NodeHandle _nh;
    ros::Publisher _pubModelJointState;
    ros::Publisher _pubControllerCommand;
    ros::Publisher _pubOdometry;
    ros::Publisher _pubSupport;
    tf2_ros::TransformBroadcaster _odom_broadcaster;
    ros::Subscriber _subCmdVel;
    ros::Subscriber _subRobState;
    ros::Subscriber _subJointStates;
    ros::Subscriber _subKick;
    ros::Subscriber _subImu;

    bool _kick;

    ros::Publisher _pubDebug;
    ros::Publisher _pubDebugMarker;

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

    bool _ik_x_offset;

};

#endif
