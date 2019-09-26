#ifndef BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
#define BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace bitbots_quintic_walk {

enum WalkState {
  PAUSED,
  WALKING,
  IDLE,
  START_MOVEMENT,
  STOP_MOVEMENT,
  START_STEP,
  STOP_STEP,
  KICK
};

struct WalkRequest {
  tf2::Vector3 orders;
  bool walkable_state;
};

struct WalkResponse {
  tf2::Transform support_foot_to_flying_foot;
  tf2::Transform support_foot_to_trunk;
  bool is_double_support;
  bool is_left_support_foot;

  double phase;
  double traj_time;
  double foot_distance;

  WalkState state;

  tf2::Transform support_to_last_;
  tf2::Transform support_to_next_;
};

/**
 * Return the given angle in radian
 * bounded between -PI and PI
 */
inline double AngleBound(double angle)
{
  return
      angle
          - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

/**
 * Compute the oriented distance between the two given angle
 * in the range -PI/2:PI/2 radian from angleSrc to angleDst
 * (Better than doing angleDst-angleSrc)
 */
inline double AngleDistance(double angleSrc, double angleDst)
{
  angleSrc = AngleBound(angleSrc);
  angleDst = AngleBound(angleDst);

  double max, min;
  if (angleSrc > angleDst) {
    max = angleSrc;
    min = angleDst;
  } else {
    max = angleDst;
    min = angleSrc;
  }

  double dist1 = max-min;
  double dist2 = 2.0*M_PI - max + min;

  if (dist1 < dist2) {
    if (angleSrc > angleDst) {
      return -dist1;
    } else {
      return dist1;
    }
  } else {
    if (angleSrc > angleDst) {
      return dist2;
    } else {
      return -dist2;
    }
  }
}
}

#endif //BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
