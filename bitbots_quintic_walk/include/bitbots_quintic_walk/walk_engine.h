/*
This code is partly based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_ENGINE_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "bitbots_splines/abstract_engine.h"
#include "bitbots_splines/smooth_spline.h"
#include "bitbots_splines/pose_spline.h"

#include "bitbots_quintic_walk/walk_utils.h"

namespace bitbots_quintic_walk {

struct EngineParams{
  // Full walk cycle frequency (in Hz, > 0) range: [0.1,5]
  double freq;
  // Length of double support phase in half cycle(ratio, [0:1]) range: [0,1]
  double double_support_ratio;
  // Lateral distance between the feet center (in m, >= 0) range: [0,1]
  double foot_distance;
  // Maximum flying foot height (in m, >= 0) range: [0,2]
  double foot_rise;
  // Trunk lateral oscillation amplitude ratio (ratio, >= 0) range: [0,2]
  double trunk_swing;
  // Height of the trunk from ground (in m, > 0) range: [0,1]
  double trunk_height;
  // Trunk pitch orientation (in rad) range: [-1,1]
  double trunk_pitch;
  // Trunk pitch orientation proportional to forward/backward step (in rad/m) range: [0,20]
  double trunk_pitch_p_coef_forward;
  // Phase offset of trunk oscillation (half cycle phase, [-1:1]) range: [-1,1]
  double trunk_phase;
  // Pause of Z movement on highest point (single support cycle ratio, [0,1]) range: [0,1]
  double foot_z_pause;
  // Let the foot's downward trajectory end above the ground this is helpful if the support leg bends, (in m, >= 0)) range: [0,0.1]
  double foot_put_down_z_offset;
  // Phase time for moving the foot from Z offset to ground (phase between apex and single support end [0:1]) range: [0,1]
  double foot_put_down_phase;
  // Phase of flying foot apex(single support cycle phase, [0:1]) range: [0,1]
  double foot_apex_phase;
  // Foot X/Y overshoot in ratio of step length(ratio, >= 0) range: [0,1]
  double foot_overshoot_ratio;
  // Foot X/Y overshoot phase (single support cycle phase, [foot_apex_phase:1] range: [0,1]
  double foot_overshoot_phase;
  // Trunk forward offset (in m) range: [-0.2,0.2]
  double trunk_x_offset;
  // Trunk lateral offset (in m) range: [-0.2,0.2]
  double trunk_y_offset;
  // Trunk swing pause length in phase at apex (half cycle ratio, [0:1]) range: [0,1]
  double trunk_pause;
  // Trunk forward offset proportional to forward step (in 1) range: [0,1]
  double trunk_x_offset_p_coef_forward;
  // Trunk forward offset proportional to rotation step (in m/rad) range: [0,1]
  double trunk_x_offset_p_coef_turn;
  // Trunk pitch orientation proportional to rotation step (in 1) range: [-20,20]
  double trunk_pitch_p_coef_turn;
  // Length of kick movement [m] range: [0,1]
  double kick_length;
  // vel kick [m/s] range: [0,100]
  double kick_vel;
  // Time of kick apex [ratio of single support phase] range: [0,1]
  double kick_phase;
  // Roll offset on flying foot at put down [rad] range: [-1,1]
  double foot_put_down_roll_offset;
  // Give extra swing to first step for better start range: [0,10]
  double first_step_swing_factor;
  // Trunk phase for the fist step range: [-1,1]
  double first_step_trunk_phase;
  // Amount of movement in z direction for trunk (around trunk_height) [m] range: [0.0,0.1]
  double trunk_z_movement;
};

/**
 * QuinticWalk
 *
 * Holonomic and open loop walk generator based on footstep control and quintic splines in cartesian space.
 * Expressed all target state in cartesian space with respect to current support foot.
 */
class WalkEngine : public bitbots_splines::AbstractEngine<WalkRequest, WalkResponse> {
 public:
  explicit WalkEngine(rclcpp::Node::SharedPtr node);

  // methods from abstract engine class
  WalkResponse update(double dt) override;
  void setGoals(const WalkRequest &goals) override;
  void reset() override;
  [[nodiscard]] int getPercentDone() const override;

  /**
   * Resets the engine to any given state. Necessary for using it as reference in learning.
   */
  void reset(WalkState state,
             double phase,
             std::vector<double> step,
             bool stop_walk,
             bool walkable_state,
             bool reset_odometry);

  /**
   * Return current walk phase between 0 and 1
   */
  [[nodiscard]] double getPhase() const;

  /**
    * Returns the phase of one single step, on which we can start doing phase resets.
      Basically the phase when the flying foot reached its apex.
    */

  [[nodiscard]] double getPhaseResetPhase() const;

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

  tf2::Transform getLeft();
  tf2::Transform getRight();

  EngineParams params_;
  bool onSetParameters(const rclcpp::Parameter & parameter);

 private:

  rclcpp::Node::SharedPtr node_;

  WalkState engine_state_;

  WalkRequest request_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

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
  double pause_duration_;
  bool pause_requested_;

  // phase rest
  bool phase_rest_active_;

  // kick handling
  bool left_kick_requested_;
  bool right_kick_requested_;

  // Current support foot (left or right).
  bool is_left_support_foot_;

  // Pose diff [dx, dy, dtheta] from support foot to flying foot last and next position.
  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;

  // Pose integration of left and right foot in initial frame.
  // Set at "future" state taking into account next expected fot pose.
  tf2::Transform left_in_world_;
  tf2::Transform right_in_world_;

  //Trunk pose and orientation position, velocity and acceleration at last half step start.
  tf2::Vector3 trunk_pos_at_foot_change_;
  tf2::Vector3 trunk_pos_vel_at_foot_change_;
  tf2::Vector3 trunk_pos_acc_at_foot_change_;
  tf2::Vector3 trunk_orientation_pos_at_last_foot_change_;
  tf2::Vector3 trunk_orientation_vel_at_last_foot_change_;
  tf2::Vector3 trunk_orientation_acc_at_foot_change_;

  //Foot pose and orientation position, velocity and acceleration at last half step start.
  tf2::Vector3 foot_pos_at_foot_change_;
  tf2::Vector3 foot_pos_vel_at_foot_change_;
  tf2::Vector3 foot_pos_acc_at_foot_change_;
  tf2::Vector3 foot_orientation_pos_at_last_foot_change_;
  tf2::Vector3 foot_orientation_vel_at_last_foot_change_;
  tf2::Vector3 foot_orientation_acc_at_foot_change_;

  void updatePhase(double dt);

  void buildNormalTrajectories();

  void buildKickTrajectories();

  void buildStartMovementTrajectories();

  void buildStartStepTrajectories();

  void buildStopStepTrajectories();

  void buildStopMovementTrajectories();

  void buildTrajectories(bool start_movement, bool start_step, bool kick_step, bool stop_step);

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
  void stepFromOrders(const tf2::Vector3 &linear_orders, double angular_z);

  /**
   * Small helper method to get euler angle instead of quaternion.
   */
  tf2::Vector3 getLastEuler();
  tf2::Vector3 getNextEuler();

};

} // namespace bitbots_quintic_walk

#endif
