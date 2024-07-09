/*
This code is partly based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <rclcpp/rclcpp.hpp>

#include "bitbots_quintic_walk/walk_utils.hpp"
#include "bitbots_quintic_walk_parameters.hpp"
#include "bitbots_splines/abstract_engine.hpp"
#include "bitbots_splines/pose_spline.hpp"
#include "bitbots_splines/smooth_spline.hpp"

namespace bitbots_quintic_walk {

/**
 * QuinticWalk
 *
 * Holonomic and open loop walk generator based on footstep control and quintic splines in cartesian space.
 * Expressed all target state in cartesian space with respect to current support foot.
 */
class WalkEngine : public bitbots_splines::AbstractEngine<WalkRequest, WalkResponse> {
 public:
  explicit WalkEngine(rclcpp::Node::SharedPtr node, walking::Params::Engine config);

  // methods from abstract engine class
  WalkResponse update(double dt) override;
  void setGoals(const WalkRequest &goals) override;
  void reset() override;
  [[nodiscard]] int getPercentDone() const override;

  /**
   * Updates the engine configuration.
   */
  void setConfig(walking::Params::Engine config);

  /**
   * Resets the engine to any given state. Necessary for using it as reference in learning.
   */
  void reset(WalkState state, double phase, std::array<double, 4> step, bool stop_walk, bool walkable_state,
             bool reset_odometry);

  /**
   * Return current walk phase between 0 and 1
   */
  [[nodiscard]] double getPhase() const;

  /**
   * Return current time between 0 and half period for trajectories evaluation
   */
  [[nodiscard]] double getTrajsTime() const;

  /**
   * Return if true if left is current support foot
   */
  [[nodiscard]] bool isLeftSupport() const;

  /**
   * Return true if both feet are currently on the ground
   */
  bool isDoubleSupport();

  void requestKick(bool left);

  void requestPause();

  /**
   * Ends the current step earlier. Useful if foot hits ground to early.
   */
  void endStep();

  void setPhaseRest(bool active);

  WalkState getState();

  [[nodiscard]] double getFreq() const;

  double getWantedTrunkPitch();

  void setPauseDuration(double duration);
  void setForceSmoothStepTransition(bool force);

  tf2::Transform getLeft();
  tf2::Transform getRight();

  walking::Params::Engine config_;

 private:
  rclcpp::Node::SharedPtr node_;

  WalkState engine_state_ = WalkState::IDLE;

  WalkRequest request_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // splines
  bitbots_splines::SmoothSpline is_double_support_spline_;
  bitbots_splines::SmoothSpline is_left_support_foot_spline_;
  bitbots_splines::PoseSpline trunk_spline_;
  bitbots_splines::PoseSpline foot_spline_;

  // Movement phase between 0 and 1
  double phase_ = 0.0;
  double last_phase_ = 0.0;

  // pause handling
  double time_paused_ = 0.0;
  double pause_duration_ = 0.0;
  bool pause_requested_ = false;

  // phase rest
  bool phase_rest_active_ = false;

  // kick handling
  bool left_kick_requested_ = false;
  bool right_kick_requested_ = false;

  // Current support foot (left or right).
  bool is_left_support_foot_ = false;

  // forces smooth spline transition if a low engine rate is used
  bool force_smooth_step_transition_ = false;

  // Pose diff [dx, dy, dtheta] from support foot to flying foot last and next position.
  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;

  // Pose integration of left and right foot in initial frame.
  // Set at "future" state taking into account next expected fot pose.
  tf2::Transform left_in_world_;
  tf2::Transform right_in_world_;

  // Trunk pose and orientation position, velocity and acceleration at last half step start.
  tf2::Vector3 trunk_pos_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 trunk_pos_vel_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 trunk_pos_acc_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 trunk_orientation_pos_at_last_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 trunk_orientation_vel_at_last_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 trunk_orientation_acc_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);

  // Foot pose and orientation position, velocity and acceleration at last half step start.
  tf2::Vector3 foot_pos_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 foot_pos_vel_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 foot_pos_acc_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 foot_orientation_pos_at_last_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 foot_orientation_vel_at_last_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 foot_orientation_acc_at_foot_change_ = tf2::Vector3(0.0, 0.0, 0.0);

  enum TrajectoryType { NORMAL, KICK, START_MOVEMENT, START_STEP, STOP_STEP, STOP_MOVEMENT };

  void updatePhase(double dt);

  void buildTrajectories(TrajectoryType type);

  void buildWalkDisableTrajectories(bool foot_in_idle_position);

  void saveCurrentRobotState();

  /**
   * Compute current cartesian target from trajectories and assign it to given model through inverse kinematics.
   * Return false is the target is unreachable.
   */
  WalkResponse createResponse();

  /**
   * Set the target pose of current support foot during next support phase and update support foot.
   * The target foot pose diff is given with respect to next support foot pose (current flying foot target).
   */
  void stepFromSupport(const tf2::Transform &diff);

  /**
   * Set target pose of current support foot using diff orders.
   * Zero vector means in place walking.
   * Special handle of lateral and turn step to avoid foot collision.
   */
  void stepFromOrders(const std::vector<double> &linear_orders, double angular_z);

  /**
   * Small helper method to get euler angle instead of quaternion.
   */
  tf2::Vector3 getLastEuler();
  tf2::Vector3 getNextEuler();
};

}  // namespace bitbots_quintic_walk

#endif
