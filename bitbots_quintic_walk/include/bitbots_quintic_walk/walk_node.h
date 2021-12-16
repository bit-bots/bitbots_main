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
#include <humanoid_league_msgs/msg/robot_control_state.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_msgs/msg/support_state.hpp>

#include <dynamic_reconfigure/server.h>
#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/msg/vector3.hpp>
#include <tf2/LinearMath/msg/quaternion.hpp>
#include <tf2/LinearMath/msg/transform.hpp>
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

class WalkNode {
 public:
  explicit WalkNode(const std::string ns);
  bitbots_msgs::msg::JointCommand step(double dt);
  bitbots_msgs::msg::JointCommand step(
      double dt,
      const geometry_msgs::msg::Twist &cmdvel_msg,
      const sensor_msgs::msg::Imu &imu_msg,
      const sensor_msgs::msg::JointState &jointstate_msg,
      const bitbots_msgs::msg::FootPressure &pressure_left,
      const bitbots_msgs::msg::FootPressure &pressure_right);
  geometry_msgs::msg::PoseArray step_open_loop(double dt, const geometry_msgs::msg::Twist &cmdvel_msg);

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
  void reset(WalkState state, double phase, geometry_msgs::msg::Twist cmd_vel, bool reset_odometry);

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

  /**
   * Sets the current state of the robot
   * @param msg The current state
   */
  void robotStateCb(humanoid_league_msgs::RobotControlState msg);

  WalkEngine *getEngine();

  nav_msgs::msg::Odometry getOdometry();

 private:
  void publishGoals(const bitbots_splines::JointGoals &goals);

  void publishOdometry(WalkResponse response);

  std::vector<double> get_step_from_vel(const geometry_msgs::msg::Twist msg);
  void stepCb(const geometry_msgs::msg::Twist msg);
  void cmdVelCb(geometry_msgs::msg::Twist msg);

  void imuCb(const sensor_msgs::msg::Imu &msg);

  void checkPhaseRestAndReset();
  void pressureRightCb(bitbots_msgs::msg::FootPressure msg);
  void pressureLeftCb(bitbots_msgs::msg::FootPressure msg);

  void jointStateCb(const sensor_msgs::msg::JointState &msg);

  void kickCb(const std_msgs::msg::BoolConstPtr &msg);

  void copLeftCb(geometry_msgs::msg::PointStamped msg);

  void copRightCb(geometry_msgs::msg::PointStamped msg);

  /**
   * This method computes the next motor goals and publishes them.
   */
  void calculateAndPublishJointGoals(const WalkResponse &response, double dt);

  double getTimeDelta();

  std::string odom_frame_, base_link_frame_, l_sole_frame_, r_sole_frame_;

  WalkRequest current_request_;

  bool debug_active_;
  bool simulation_active_;

  bool first_run_;

  double engine_frequency_;

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

  bitbots_quintic_walk_paramsConfig params_;

  /**
   * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
   * Is used to limit _currentOrders to sane values
   */
  Eigen::msg::Vector3d max_step_linear_;
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

  
  

  ros::Publisher pub_controller_command_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_support_;
  tf2_ros::msg::TransformBroadcaster odom_broadcaster_;

  ros::Subscriber step_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber robot_state_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber kick_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber pressure_sub_left_;
  ros::Subscriber pressure_sub_right_;

  dynamic_reconfigure::Server<bitbots_quintic_walk_paramsConfig> *dyn_reconf_server_;

  // MoveIt!
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::msg::RobotStatePtr current_state_;

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
