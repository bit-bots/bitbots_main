/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef QUINTICWALK_HPP
#define QUINTICWALK_HPP

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Footstep.hpp"
#include "bitbots_quintic_walk/TrajectoryUtils.h"
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
class QuinticWalk : public AbstractEngine<WalkPositions, WalkGoals> {
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
  void reconf_callback(const bitbots_quintic_walk_paramsConfig &params);

  /**
   * Update the internal walk state
   * (pĥase, trajectories) from given
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
  void computeCartesianPosition(Eigen::Vector3d &trunkPos, Eigen::Vector3d &trunkAxis,
                                Eigen::Vector3d &footPos, Eigen::Vector3d &footAxis, bool &isLeftsupportFoot);

  void
  computeCartesianPositionAtTime(Eigen::Vector3d &trunkPos, Eigen::Vector3d &trunkAxis, Eigen::Vector3d &footPos,
                                 Eigen::Vector3d &footAxis, bool &isLeftsupportFoot, double time);

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

  std::string _engineState;

  /**
   * Current footstep support
   * and flying last and next pose
   */
  Footstep _footstep;

  /**
   * Movement phase between 0 and 1
   */
  double _phase;
  double _lastPhase;

  double _timePaused;

  /**
   * Currently used parameters
   */
  bitbots_quintic_walk_paramsConfig _params;

  bool _leftKickRequested;
  bool _rightKickRequested;
  bool _pauseRequested;

  /**
   * Trunk pose and orientation
   * position, velocity and acceleration
   * at half cycle start
   */
  Eigen::Vector3d _trunkPosAtLast;
  Eigen::Vector3d _trunkVelAtLast;
  Eigen::Vector3d _trunkAccAtLast;
  Eigen::Vector3d _trunkAxisPosAtLast;
  Eigen::Vector3d _trunkAxisVelAtLast;
  Eigen::Vector3d _trunkAxisAccAtLast;

  /**
   * Generated half walk
   * cycle trajectory
   */
  Trajectories _trajs;

  void updatePhase(double dt);

  void buildNormalTrajectories(const Eigen::Vector3d &orders);

  void buildKickTrajectories(const Eigen::Vector3d &orders);

  void buildStartTrajectories(const Eigen::Vector3d &orders);

  void buildStopStepTrajectories(const Eigen::Vector3d &orders);

  void buildStopMovementTrajectories(const Eigen::Vector3d &orders);

  void buildTrajectories(const Eigen::Vector3d &orders, bool start_movement, bool start_step, bool kick_step);

  void buildWalkDisableTrajectories(const Eigen::Vector3d &orders, bool footInIdlePosition);

  void saveCurrentTrunkState();

  void useCurrentTrunkState();

  void point(std::string spline, double t, double pos, double vel = 0, double acc = 0);

  /**
   * Reset the trunk position and
   * orientation state vectors at last
   * half cycle as stopped pose
   */
  void resetTrunkLastState();
};

}

#endif
