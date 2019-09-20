/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_

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
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_msgs/FootPressure.h>

#include <dynamic_reconfigure/server.h>
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
#include "bitbots_quintic_walk/walk_engine.h"
#include <std_msgs/Bool.h>
#include <unistd.h>
#include "bitbots_quintic_walk/walk_stabilizer.h"
#include "bitbots_quintic_walk/walk_ik.h"
#include "bitbots_splines/abstract_ik.h"
#include "bitbots_quintic_walk/walk_visualizer.h"

namespace bitbots_quintic_walk {

class WalkNode {
 public:
  WalkNode();

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
  void reconfCallback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

  /**
   * Initialize internal WalkEngine to correctly zeroed, usable state
   */
  void initializeEngine();

 private:
  void publishGoals(const bitbots_splines::JointGoals &goals);

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

  void copLCb(const geometry_msgs::PointStamped msg);

  void copRCb(const geometry_msgs::PointStamped msg);

  /**
   * This method computes the next motor goals and publishes them.
   */
  void calculateAndPublishJointGoals(WalkResponse response);

  double getTimeDelta();

  WalkRequest current_request_;

  bool debug_active_;
  bool simulation_active_;

  bool first_run_;

  double engine_frequency_;

  bool phase_reset_active_;
  double phase_reset_phase_;
  double ground_min_pressure_;
  bool cop_stop_active_;
  double cop_x_threshold_;
  double cop_y_threshold_;
  bool pressure_stop_active_;
  double io_pressure_threshold_;
  double fb_pressure_threshold_;

  bool imu_active_;
  double imu_pitch_threshold_;
  double imu_roll_threshold_;
  double imu_pitch_vel_threshold_;
  double imu_roll_vel_threshold_;

  bool publish_odom_tf_;
  int odom_pub_factor_;
  double last_ros_update_time_;

  int robot_state_;

  bitbots_quintic_walk_paramsConfig params_;

  /**
   * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
   * Is used to limit _currentOrders to sane values
   */
  Eigen::Vector3d max_step_;

  /**
   * Measures how much distance we can traverse in X and Y direction combined
   */
  double max_step_xy_;
  bitbots_quintic_walk::WalkEngine walk_engine_;

  bitbots_msgs::JointCommand command_msg_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::TransformStamped odom_trans_;

  ros::NodeHandle nh_;

  ros::Publisher pub_controller_command_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_support_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;

  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_rob_state_;
  ros::Subscriber sub_joint_states_;
  ros::Subscriber sub_kick_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_pressure_;
  ros::Subscriber sub_cop_l_;
  ros::Subscriber sub_cop_r_;

  dynamic_reconfigure::Server<bitbots_quintic_walk_paramsConfig> server_;

  geometry_msgs::PointStamped cop_l_;
  geometry_msgs::PointStamped cop_r_;

  // MoveIt!
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr current_state_;

  WalkStabilizer stabilizer_;
  WalkIK ik_;
  WalkVisualizer visualizer_;

};

}

#endif
