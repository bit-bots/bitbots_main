/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_FOOTSTEP_HPP_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_FOOTSTEP_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include "bitbots_splines/Angle.h"

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
  Footstep(
      double foot_distance,
      bool is_left_support_foot = true);

  /**
   * Set the lateral foot
   * distance parameters
   */
  void setfoot_distance(double foot_distance);
  double getfoot_distance();

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
  const Eigen::Vector3d &getLast() const;

  /**
   * Target pose of current flying
   * foot in support foot frame
   */
  const Eigen::Vector3d &getNext() const;

  /**
   * Returns the odometry change of the current step.
   */
  const Eigen::Vector3d &getOdom() const;

  /**
   * Left and right, current or next pose
   * of foot in world initial frame
   */
  const Eigen::Vector3d &getLeft() const;
  const Eigen::Vector3d &getRight() const;

  /**
   * Set the target pose of current support foot
   * during next support phase and update support foot.
   * The target foot pose diff is given with respect to
   * next support foot pose (current flying foot target).
   */
  void stepFromSupport(const Eigen::Vector3d &diff);

  /**
   * Set target pose of current support foot
   * using diff orders.
   * Zero vector means in place walking.
   * Special handle of lateral and turn step
   * to avoid foot collision.
   */
  void stepFromOrders(const Eigen::Vector3d &diff);

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
  Eigen::Vector3d support_to_last_;
  Eigen::Vector3d support_to_next_;

  /**
   * Pose integration of left
   * and right foot in initial frame.
   * Set at "future" state taking into account
   * next expected fot pose.
   */
  Eigen::Vector3d left_in_world_;
  Eigen::Vector3d right_in_world_;

  /**
   * Add to given pose the given diff
   * expressed in pose frame and
   * return the integrated added pose
   */
  Eigen::Vector3d poseAdd(
      const Eigen::Vector3d &pose,
      const Eigen::Vector3d &diff) const;

  /**
   * Compute and return the delta from
   * (zero+diff) to (zero) in
   * (zero+diff) frame.
   */
  Eigen::Vector3d diffInv(
      const Eigen::Vector3d &diff) const;
};

}
#endif