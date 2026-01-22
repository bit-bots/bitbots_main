#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_

#include <Eigen/Geometry>
#include <bitbots_dynamic_kick/msg/kick_debug.hpp>

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
  INITIAL = bitbots_dynamic_kick::msg::KickDebug::INITIAL,
  MOVE_TRUNK = bitbots_dynamic_kick::msg::KickDebug::MOVE_TRUNK,
  RAISE_FOOT = bitbots_dynamic_kick::msg::KickDebug::RAISE_FOOT,
  WINDUP = bitbots_dynamic_kick::msg::KickDebug::WINDUP,
  KICK = bitbots_dynamic_kick::msg::KickDebug::KICK,
  MOVE_BACK = bitbots_dynamic_kick::msg::KickDebug::MOVE_BACK,
  LOWER_FOOT = bitbots_dynamic_kick::msg::KickDebug::LOWER_FOOT,
  MOVE_TRUNK_BACK = bitbots_dynamic_kick::msg::KickDebug::MOVE_TRUNK_BACK,
  DONE = bitbots_dynamic_kick::msg::KickDebug::DONE
};

}  // namespace bitbots_dynamic_kick

#endif  // BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_UTILS_H_
