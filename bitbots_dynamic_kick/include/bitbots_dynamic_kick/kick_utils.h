#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_

#include <Eigen/Geometry>
#include <bitbots_dynamic_kick/KickDebug.h>

namespace bitbots_dynamic_kick {

struct KickPositions {
  bool is_left_kick = true;
  Eigen::Isometry3d trunk_pose;
  Eigen::Isometry3d flying_foot_pose;
  bool cop_support_point = false;
  double engine_time;
};

struct KickGoals {
  Eigen::Vector3d ball_position;
  Eigen::Quaterniond kick_direction;
  double kick_speed = 0;
  Eigen::Isometry3d trunk_to_base_footprint;
};

enum KickPhase {
  INITIAL = KickDebug::INITIAL,
  MOVE_TRUNK = KickDebug::MOVE_TRUNK,
  RAISE_FOOT = KickDebug::RAISE_FOOT,
  WINDUP = KickDebug::WINDUP,
  KICK = KickDebug::KICK,
  MOVE_BACK = KickDebug::MOVE_BACK,
  LOWER_FOOT = KickDebug::LOWER_FOOT,
  MOVE_TRUNK_BACK = KickDebug::MOVE_TRUNK_BACK,
  DONE = KickDebug::DONE
};

}

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
