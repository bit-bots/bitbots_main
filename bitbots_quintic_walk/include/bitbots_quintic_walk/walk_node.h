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
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include "humanoid_league_msgs/msg/robot_control_state.hpp"
#include "bitbots_msgs/msg/joint_command.hpp"
#include "bitbots_msgs/msg/foot_pressure.hpp"
#include "bitbots_msgs/msg/support_state.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "bitbots_quintic_walk/walk_engine.h"
#include "bitbots_quintic_walk/walk_stabilizer.h"
#include "bitbots_quintic_walk/walk_ik.h"
#include "bitbots_splines/abstract_ik.h"
#include "bitbots_quintic_walk/walk_visualizer.h"

namespace bitbots_quintic_walk {

class WalkNode : public rclcpp::Node {
 public:
  explicit WalkNode(std::string ns);
  bitbots_msgs::msg::JointCommand step(double dt);
  bitbots_msgs::msg::JointCommand step(double dt,
                                       geometry_msgs::msg::Twist::SharedPtr cmdvel_msg,
                                       sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                       sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
                                       bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
                                       bitbots_msgs::msg::FootPressure::SharedPtr pressure_right);
  bitbots_msgs::msg::JointCommand step_relative(double dt,
                                                geometry_msgs::msg::Twist::SharedPtr step_msg,
                                                sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                                sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
                                                bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
                                                bitbots_msgs::msg::FootPressure::SharedPtr pressure_right);
  geometry_msgs::msg::PoseArray step_open_loop(double dt, geometry_msgs::msg::Twist::SharedPtr cmdvel_msg);

  /**
   * Small helper method to get foot position via python wrapper
   */
  geometry_msgs::msg::Pose get_right_foot_pose();
  geometry_msgs::msg::Pose get_left_foot_pose();

  /**
   * Reset everything to initial idle state.
   */
  void reset();

  /**
   * Reset walk to any given state. Necessary for using this as reference in learning.
   */
  void reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool reset_odometry);

  /**
   * This is the main loop which takes care of stopping and starting of the walking.
   * A small state machine is tracking in which state the walking is and builds the trajectories accordingly.
   */
  void run();

  /**
   * Initialize internal WalkEngine to correctly zeroed, usable state
   */
  void initializeEngine();

  /**
   * Sets the current state of the robot
   * @param msg The current state
   */
  void robotStateCb(humanoid_league_msgs::msg::RobotControlState::SharedPtr msg);

  WalkEngine *getEngine();

  nav_msgs::msg::Odometry getOdometry();

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> & parameters);

 private:
  std::vector<double> get_step_from_vel(geometry_msgs::msg::Twist::SharedPtr msg);
  void stepCb(geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelCb(geometry_msgs::msg::Twist::SharedPtr msg);

  void imuCb(sensor_msgs::msg::Imu::SharedPtr msg);

  void checkPhaseRestAndReset();
  void pressureRightCb(bitbots_msgs::msg::FootPressure::SharedPtr msg);
  void pressureLeftCb(bitbots_msgs::msg::FootPressure::SharedPtr msg);

  void jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg);

  void kickCb(std_msgs::msg::Bool::SharedPtr msg);

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  double getTimeDelta();

  std::string odom_frame_, base_link_frame_, l_sole_frame_, r_sole_frame_;

  WalkRequest current_request_;

  bool debug_active_;
  bool simulation_active_;

  bool first_run_;

  double engine_frequency_;

  bool phase_reset_active_;
  bool pressure_phase_reset_active_;
  bool effort_phase_reset_active_;
  double phase_reset_phase_;
  double ground_min_pressure_;
  double joint_min_effort_;
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

  int odom_pub_factor_;
  double last_ros_update_time_;

  int robot_state_;

  char current_support_foot_;

  WalkResponse current_response_;
  WalkResponse current_stabilized_response_;
  bitbots_splines::JointGoals motor_goals_;

  /**
   * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
   * Is used to limit _currentOrders to sane values
   */
  Eigen::Vector3d max_step_linear_;
  double max_step_angular_;

  /**
   * Measures how much distance we can traverse in X and Y direction combined
   */
  double max_step_xy_;
  bitbots_quintic_walk::WalkEngine walk_engine_;

  double x_speed_multiplier_;
  double y_speed_multiplier_;
  double yaw_speed_multiplier_;

  bitbots_msgs::msg::JointCommand command_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped odom_trans_;

  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<bitbots_msgs::msg::SupportState>::SharedPtr pub_support_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr step_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<humanoid_league_msgs::msg::RobotControlState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_left_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_right_;

  // MoveIt!
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr current_state_;

  WalkStabilizer stabilizer_;
  WalkIK ik_;
  WalkVisualizer visualizer_;

  double current_trunk_fused_pitch_;
  double current_trunk_fused_roll_;

  double current_fly_pressure_;
  double current_fly_effort_;

  double roll_vel_;
  double pitch_vel_;

  bool got_new_goals_;

};

} // namespace bitbots_quintic_walk

#endif // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_
