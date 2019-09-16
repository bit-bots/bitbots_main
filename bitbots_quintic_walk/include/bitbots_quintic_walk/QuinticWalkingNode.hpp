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
#include <geometry_msgs/PointStamped.h>
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
#include <std_msgs/Bool.h>
#include <unistd.h>


namespace bitbots_quintic_walk {

    class QuinticWalkingNode {
    public:
        QuinticWalkingNode();

        /**
         * This is the main loop which takes care of stopping and starting of the walking.
         * A small state machine is tracking in which state the walking is and builds the trajectories accordingly.
         */
        void run();

        /**
         * Dynamic reconfigure callback. Takes in new parameters and applies them to the needed software parts
         * @param config New configuration
         * @param level Number which represents which classes of configuration options were changed.
         *      Each parameter can be defined with a level in the .cfg file. The levels of all changed values then
         *      get bitwise ORed and passed to this callback
         */
        void reconf_callback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

        /**
         * Initialize internal WalkEngine to correctly zeroed, usable state
         */
        void initializeEngine();

    private:
        /**
         * Publish bitbots_msgs/JointCommand message to the correct topic
         * @param joint_names Names of joints which should be set to new positions
         * @param positions Target positions of the previously mentioned joints
         */
        void publishControllerCommands(std::vector<std::string> joint_names, std::vector<double> positions);

        void publishDebug(tf2::Transform &trunk_to_support_foot, tf2::Transform &trunk_to_flying_foot);

        void publishMarker(std::string name_space,
                           std::string frame,
                           geometry_msgs::Pose pose,
                           float r, float g, float b, float a);

        void publishMarkers();

        void publishOdometry();

        void cmdVelCb(geometry_msgs::Twist msg);

        void imuCb(sensor_msgs::Imu msg);

        void pressureCb(bitbots_msgs::FootPressure msg);

        void robStateCb(humanoid_league_msgs::RobotControlState msg);

        void jointStateCb(sensor_msgs::JointState msg);

        void kickCb(std_msgs::BoolConstPtr msg);

        void cop_l_cb(const geometry_msgs::PointStamped msg);

        void cop_r_cb(const geometry_msgs::PointStamped msg);

        /**
         * This method computes the next motor goals and publishes them.
         */
        void calculateJointGoals();

        double getTimeDelta();

        bool _debugActive;
        bool _simulation_active;

        bool _first_run;

        double _engineFrequency;

        bool _phaseResetActive;
        double _phaseResetPhase;
        double _groundMinPressure;
        bool _copStopActive;
        double _copXThreshold;
        double _copYThreshold;
        bool _pressureStopActive;
        double _ioPressureThreshold;
        double _fbPressureThreshold;

        bool _imuActive;
        double _imu_pitch_threshold;
        double _imu_roll_threshold;
        double _imu_pitch_vel_threshold;
        double _imu_roll_vel_threshold;


        bool _publishOdomTF;
        int _odomPubFactor;
        std::chrono::time_point<std::chrono::steady_clock> _last_update_time;
        double _last_ros_update_time;

        int _robotState;
        int _marker_id;

        bitbots_quintic_walk_paramsConfig _params;

        Eigen::Vector3d _trunkPos;
        Eigen::Vector3d _trunkAxis;
        Eigen::Vector3d _footPos;
        Eigen::Vector3d _footAxis;
        bool _isLeftSupport;

        /**
         * Saves current orders as [x-direction, y-direction, z-rotation]
         */
        Eigen::Vector3d _currentOrders;

        /**
         * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
         * Is used to limit _currentOrders to sane values
         */
        Eigen::Vector3d _max_step;

        /**
         * Measures how much distance we can traverse in X and Y direction combined
         */
        double _max_step_xy;
        bitbots_quintic_walk::QuinticWalk _walkEngine;

        bitbots_msgs::JointCommand _command_msg;
        nav_msgs::Odometry _odom_msg;
        geometry_msgs::TransformStamped _odom_trans;

        ros::NodeHandle _nh;

        ros::Publisher _pubControllerCommand;
        ros::Publisher _pubOdometry;
        ros::Publisher _pubSupport;
        tf2_ros::TransformBroadcaster _odom_broadcaster;
        ros::Publisher _pubDebug;
        ros::Publisher _pubDebugMarker;

        ros::Subscriber _subCmdVel;
        ros::Subscriber _subRobState;
        ros::Subscriber _subJointStates;
        ros::Subscriber _subKick;
        ros::Subscriber _subImu;
        ros::Subscriber _subPressure;
        ros::Subscriber _subCopL;
        ros::Subscriber _subCopR;

        dynamic_reconfigure::Server<bitbots_quintic_walk_paramsConfig> _server;

        geometry_msgs::PointStamped _cop_l;
        geometry_msgs::PointStamped _cop_r;


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

}

#endif
