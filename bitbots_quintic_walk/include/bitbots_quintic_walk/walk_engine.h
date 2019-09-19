/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "footstep.h"
#include <eigen_conversions/eigen_msg.h>
#include "bitbots_splines/SplineContainer.hpp"
#include "bitbots_splines/AxisAngle.h"
#include "bitbots_splines/Angle.h"
#include "bitbots_splines/Euler.h"
#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
#include <math.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include "bitbots_splines/SmoothSpline.hpp"
#include "bitbots_splines/pose_spline.h"
#include "bitbots_splines/position_spline.h"
#include "bitbots_quintic_walk/walk_utils.h"
#include "bitbots_splines/abstract_engine.h"

namespace bitbots_quintic_walk {

/**
 * QuinticWalk
 *
 * Holonomic and open loop walk
 * generator based on footstep control
 * and quintic splines in cartesian space.
 * Expressed all target state in cartesian
 * space with respect to current cupport foot
 */
class QuinticWalk : public bitbots_splines::AbstractEngine<WalkRequest, WalkResponse> {
 public:

  /**
   * Initialization
   */
  QuinticWalk();

  WalkResponse update(double dt) override;
  void setGoals(const WalkRequest &goals) override;
  void reset() override;
  Trajectories getSplines() const;
  int getPercentDone() const;

  /**
   * Return current walk phase
   * between 0 and 1
   */
  double getPhase() const;

  /**
   * Return current time between
   * 0 and half period for
   * trajectories evaluation
   */
  double getTrajsTime() const;

  /**
   * Get the footstep object.
   */
  Footstep getFootstep();

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
  void reconfCallback(const bitbots_quintic_walk_paramsConfig &params);


  /**
   * Compute current cartesian
   * target from trajectories and assign
   * it to given model through inverse
   * kinematics.
   * Return false is the target is
   * unreachable.
   */
  WalkResponse computeCartesianPositionAtTime(double time);

  void requestKick(bool left);

  void requestPause();

  /**
   * Ends the current step earlier. Useful if foot hits ground to early.
   */
  void endStep();

  std::string getState();

 private:

  WalkRequest request_;

  std::string engine_state_;

  //splines
  bitbots_splines::SmoothSpline is_double_support_;
  bitbots_splines::SmoothSpline is_left_support_foot_;
  bitbots_splines::PoseSpline trunk_;
  bitbots_splines::PoseSpline foot_;

  /**
   * Current footstep support
   * and flying last and next pose
   */
  Footstep footstep_;

  /**
   * Movement phase between 0 and 1
   */
  double phase_;
  double last_phase_;

  double time_paused_;

  /**
   * Currently used parameters
   */
  bitbots_quintic_walk_paramsConfig params_;

  bool left_kick_requested_;
  bool right_kick_requested_;
  bool pause_requested_;

  /**
   * Trunk pose and orientation
   * position, velocity and acceleration
   * at half cycle start
   */
  tf2::Vector3 trunk_pos_at_last_;
  tf2::Vector3 trunk_pos_vel_at_last_;
  tf2::Vector3 trunk_pos_acc_at_last_;
  tf2::Vector3 trunk_axis_pos_at_last_;
  tf2::Vector3 trunk_axis_vel_at_last_;
  tf2::Vector3 trunk_axis_acc_at_last_;

  /**
   * Generated half walk
   * cycle trajectory
   */

  void updatePhase(double dt);

  void buildNormalTrajectories();

  void buildKickTrajectories();

  void buildStartTrajectories();

  void buildStopStepTrajectories();

  void buildStopMovementTrajectories();

  void buildTrajectories(bool start_movement, bool start_step, bool kick_step);

  void buildWalkDisableTrajectories(bool foot_in_idle_position);

  void saveCurrentTrunkState();

  void point(bitbots_splines::SmoothSpline spline, double t, double pos, double vel = 0, double acc = 0);

  /**
   * Reset the trunk position and
   * orientation state vectors at last
   * half cycle as stopped pose
   */
  void resetTrunkLastState();
};

}

#endif
