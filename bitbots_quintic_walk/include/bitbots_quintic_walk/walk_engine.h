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
#include "bitbots_quintic_walk/trajectory_utils.h"
#include <eigen_conversions/eigen_msg.h>
#include "bitbots_splines/SplineContainer.hpp"
#include "bitbots_splines/AxisAngle.h"
#include "bitbots_splines/Angle.h"
#include "bitbots_splines/Euler.h"
#include "bitbots_quintic_walk/common.h"
#include <bitbots_quintic_walk/bitbots_quintic_walk_paramsConfig.h>
#include <math.h>

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
class QuinticWalk : public bitbots_splines::AbstractEngine<WalkPositions, WalkGoals> {
 public:

  /**
   * Initialization
   */
  QuinticWalk();

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
   * Update the internal walk state
   * (phase, trajectories) from given
   * elapsed time since last update() call
   */
  bool updateState(double dt, const Eigen::Vector3d &orders, bool walkable_state);

  /**
   * Compute current cartesian
   * target from trajectories and assign
   * it to given model through inverse
   * kinematics.
   * Return false is the target is
   * unreachable.
   */
  void computeCartesianPosition(Eigen::Vector3d &trunk_pos, Eigen::Vector3d &trunk_axis,
                                Eigen::Vector3d &foot_pos, Eigen::Vector3d &foot_axis, bool &is_leftsupport_foot);

  void
  computeCartesianPositionAtTime(Eigen::Vector3d &trunk_pos, Eigen::Vector3d &trunk_axis, Eigen::Vector3d &foot_pos,
                                 Eigen::Vector3d &foot_axis, bool &is_leftsupport_foot, double time);

  void requestKick(bool left);

  void requestPause();

  /**
   * Ends the current step earlier. Useful if foot hits ground to early.
   */
  void endStep();

  /**
   * Completely reset the engine, e.g. when robot fell down
   */
  void reset();

  std::string getState();

 private:

  std::string engine_state_;

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
  Eigen::Vector3d trunk_pos_at_last_;
  Eigen::Vector3d trunk_vel_at_last_;
  Eigen::Vector3d trunk_acc_at_last_;
  Eigen::Vector3d trunk_axis_pos_at_last_;
  Eigen::Vector3d trunk_axis_vel_at_last_;
  Eigen::Vector3d trunk_axis_acc_at_last_;

  /**
   * Generated half walk
   * cycle trajectory
   */
  Trajectories trajs_;

  void updatePhase(double dt);

  void buildNormalTrajectories(const Eigen::Vector3d &orders);

  void buildKickTrajectories(const Eigen::Vector3d &orders);

  void buildStartTrajectories(const Eigen::Vector3d &orders);

  void buildStopStepTrajectories(const Eigen::Vector3d &orders);

  void buildStopMovementTrajectories(const Eigen::Vector3d &orders);

  void buildTrajectories(const Eigen::Vector3d &orders, bool start_movement, bool start_step, bool kick_step);

  void buildWalkDisableTrajectories(const Eigen::Vector3d &orders, bool foot_in_idle_position);

  void saveCurrentTrunkState();

  void point(const std::string &spline, double t, double pos, double vel = 0, double acc = 0);

  /**
   * Reset the trunk position and
   * orientation state vectors at last
   * half cycle as stopped pose
   */
  void resetTrunkLastState();
};

}

#endif
