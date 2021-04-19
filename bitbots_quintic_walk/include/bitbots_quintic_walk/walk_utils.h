#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_

#define M_TAU M_PI * 2

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  tf2::Vector3 linear_orders = {0, 0, 0};
  double angular_z = 0;
  bool walkable_state = false;
};

struct WalkResponse {
  tf2::Transform support_foot_to_flying_foot = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
  tf2::Transform support_foot_to_trunk = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));

  // additional information for visualization
  bool is_double_support = false;
  bool is_left_support_foot = false;

  double phase = 0.0;
  double traj_time = 0.0;
  double foot_distance = 0.0;

  WalkState state = WalkState::IDLE;

  tf2::Transform support_to_last = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
  tf2::Transform support_to_next = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));

  double current_pitch = 0.0;
  double current_fused_pitch = 0.0;
  double current_roll = 0.0;
  double current_fused_roll = 0.0;

  double roll_vel = 0.0;
  double pitch_vel = 0.0;

  double sup_cop_x = 0.0;
  double sup_cop_y = 0.0;
};

/**
 * Return the given angle in radian
 * bounded between -TAU/2 and TAU/2
 */
inline double angleBound(double angle) {
  return
      angle
          - M_TAU * std::floor((angle + M_TAU / 2) / M_TAU);
}

/**
 * Compute the oriented distance between the two given angle
 * in the range -TAU/4:TAU/4 radian from angleSrc to angleDst
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
  double dist_2 = M_TAU - max + min;

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
} // namespace bitbots_quintic_walk

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
