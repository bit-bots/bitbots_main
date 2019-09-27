/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_quintic_walk/bitbots_quintic_walk_engine_paramsConfig.h>

#include "bitbots_splines/abstract_engine.h"
#include "bitbots_splines/SmoothSpline.hpp"
#include "bitbots_splines/pose_spline.h"

#include "bitbots_quintic_walk/walk_utils.h"

namespace bitbots_quintic_walk {

/**
 * QuinticWalk
 *
 * Holonomic and open loop walk generator based on footstep control and quintic splines in cartesian space.
 * Expressed all target state in cartesian space with respect to current support foot.
 */
class WalkEngine : public bitbots_splines::AbstractEngine<WalkRequest, WalkResponse> {
 public:
  WalkEngine();

  // methods from abstract engine class
  WalkResponse update(double dt) override;
  void setGoals(const WalkRequest &goals) override;
  void reset() override;
  int getPercentDone() const override;

  /**
   * Return current walk phase between 0 and 1
   */
  double getPhase() const;

  /**
   * Return current time between 0 and half period for trajectories evaluation
   */
  double getTrajsTime() const;

  /**
   * Return if true if left is current support foot
   */
  bool isLeftSupport();

  /**
   * Return true if both feet are currently on the ground
   */
  bool isDoubleSupport();

  /**
   * Assign given parameters vector
   */
  void reconfCallback(bitbots_quintic_walk_engine_paramsConfig &params, uint32_t level);

  void requestKick(bool left);

  void requestPause();

  /**
   * Ends the current step earlier. Useful if foot hits ground to early.
   */
  void endStep();

  WalkState getState();

  double getFreq();

  double getWantedTrunkPitch();

  void setPauseDuration(double duration);

  tf2::Transform getLeft();
  tf2::Transform getRight();

 private:

  WalkState engine_state_;

  WalkRequest request_;

  // Currently used parameters
  bitbots_quintic_walk_engine_paramsConfig params_;
  dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig> *dyn_reconf_server_;

  //splines
  bitbots_splines::SmoothSpline is_double_support_spline_;
  bitbots_splines::SmoothSpline is_left_support_foot_spline_;
  bitbots_splines::PoseSpline trunk_spline_;
  bitbots_splines::PoseSpline foot_spline_;

  //Movement phase between 0 and 1
  double phase_;
  double last_phase_;

  // pause handling
  double time_paused_;
  double pause_duration_{};
  bool pause_requested_;

  // kick handling
  bool left_kick_requested_;
  bool right_kick_requested_;

  // Current support foot (left or right).
  bool is_left_support_foot_{};

  // Pose diff [dx, dy, dtheta] from support foot to flying foot last and next position.
  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;

  // Pose integration of left and right foot in initial frame.
  // Set at "future" state taking into account next expected fot pose.
  tf2::Transform left_in_world_;
  tf2::Transform right_in_world_;

  //Trunk pose and orientation position, velocity and acceleration at half cycle start.
  tf2::Vector3 trunk_pos_at_foot_change_;
  tf2::Vector3 trunk_pos_vel_at_foot_change_;
  tf2::Vector3 trunk_pos_acc_at_foot_change_;
  tf2::Vector3 trunk_orientation_pos_at_last_foot_change_;
  tf2::Vector3 trunk_orientation_vel_at_last_foot_change_;
  tf2::Vector3 trunk_orientation_acc_at_foot_change_;

  void updatePhase(double dt);

  void buildNormalTrajectories();

  void buildKickTrajectories();

  void buildStartMovementTrajectories();

  void buildStartStepTrajectories();

  void buildStopStepTrajectories();

  void buildStopMovementTrajectories();

  void buildTrajectories(bool start_movement, bool start_step, bool kick_step);

  void buildWalkDisableTrajectories(bool foot_in_idle_position);

  void saveCurrentTrunkState();

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
  void stepFromOrders(const tf2::Vector3 &diff);

  /**
   * Small helper method to get euler angle instead of quaternion.
   */
  tf2::Vector3 getLastEuler();
  tf2::Vector3 getNextEuler();

};

}

#endif
