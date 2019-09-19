/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_FOOTSTEP_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_FOOTSTEP_H_

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include "bitbots_splines/Angle.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>


namespace bitbots_quintic_walk {

/**
 * Footstep
 *
 * Manage humanoid footstep
 * generation and state
 */
class Footstep {
 public:

  enum SupportFoot {
    LEFT_SUPPORT_FOOT,
    RIGHT_SUPPORT_FOOT,
  };

  /**
   * Initialization with lateral
   * foot distance and support foot
   */
  explicit Footstep(
      double foot_distance,
      bool is_left_support_foot = true);

  /**
   * Set the lateral foot
   * distance parameters
   */
  void setFootDistance(double foot_distance);
  double getFootDistance();

  /**
   * Reset to neutral position the current
   * step (not the integrated odometry)
   */
  void reset(bool is_left_support_foot);

  // reset odometry
  void resetInWorld(bool is_left_support_foot);

  /**
   * Current support foot
   */
  bool isLeftSupport() const;

  /**
   * Starting position of current flying
   * foot in support foot frame
   */
  const tf2::Transform &getLast() const;
  const tf2::Vector3 &getLastPos() const;
  tf2::Vector3 getLastEuler();

  /**
   * Target pose of current flying
   * foot in support foot frame
   */
  const tf2::Transform &getNext() const;
  const tf2::Vector3 &getNextPos() const;
  tf2::Vector3 getNextEuler();

  /**
   * Left and right, current or next pose
   * of foot in world initial frame
   */
  const tf2::Transform &getLeft() const;
  const tf2::Transform &getRight() const;

  /**
   * Set the target pose of current support foot
   * during next support phase and update support foot.
   * The target foot pose diff is given with respect to
   * next support foot pose (current flying foot target).
   */
  void stepFromSupport(const tf2::Transform &diff);

  /**
   * Set target pose of current support foot
   * using diff orders.
   * Zero vector means in place walking.
   * Special handle of lateral and turn step
   * to avoid foot collision.
   */
  void stepFromOrders(const tf2::Transform &diff);

 private:

  /**
   * Static lateral distance
   * between the feet
   */
  double foot_distance_;

  /**
   * Current support foot
   * (left or right)
   */
  bool is_left_support_foot_;

  /**
   * Pose diff [dx, dy, dtheta]
   * from support foot to flying foot
   * last and next position
   */
  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;

  /**
   * Pose integration of left
   * and right foot in initial frame.
   * Set at "future" state taking into account
   * next expected fot pose.
   */
  tf2::Transform left_in_world_;
  tf2::Transform right_in_world_;
};
}
#endif