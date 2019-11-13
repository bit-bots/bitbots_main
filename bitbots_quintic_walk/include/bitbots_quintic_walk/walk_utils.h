#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_

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

  // additional information for visualization
  bool is_double_support;
  bool is_left_support_foot;

  double phase;
  double traj_time;
  double foot_distance;

  WalkState state;

  tf2::Transform support_to_last;
  tf2::Transform support_to_next;

  double current_pitch;
};

/**
 * Return the given angle in radian
 * bounded between -PI and PI
 */
inline double angleBound(double angle) {
  return
      angle
          - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * Compute the oriented distance between the two given angle
 * in the range -PI/2:PI/2 radian from angleSrc to angleDst
 * (Better than doing angleDst-angleSrc)
 */
inline double angleDistance(double angle_src, double angle_dst) {
  angle_src = angleBound(angle_src);
  angle_dst = angleBound(angle_dst);

  double max, min;
  if (angle_src > angle_dst) {
    max = angle_src;
    min = angle_dst;
  } else {
    max = angle_dst;
    min = angle_src;
  }

  double dist_1 = max - min;
  double dist_2 = 2.0 * M_PI - max + min;

  if (dist_1 < dist_2) {
    if (angle_src > angle_dst) {
      return -dist_1;
    } else {
      return dist_1;
    }
  } else {
    if (angle_src > angle_dst) {
      return dist_2;
    } else {
      return -dist_2;
    }
  }
}
}

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
