#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_

#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <string>
#include <tf2/LinearMath/Transform.hpp>

namespace bitbots_dynup {

struct DynupResponse {
  tf2::Transform l_foot_goal_pose;  // relative to r_foot_goal_pose
  tf2::Transform r_foot_goal_pose;
  tf2::Transform l_hand_goal_pose;
  tf2::Transform r_hand_goal_pose;
  bool is_stabilizing_needed;
  bool is_head_zero;
};

enum DynupDirection {
  FRONT = 1,
  BACK = 0,
  FRONT_ONLY = 4,
  BACK_ONLY = 5,
  RISE = 2,
  DESCEND = 3,
  WALKREADY = 6,
  RISE_NO_ARMS = 8,
  DESCEND_NO_ARMS = 7
};

DynupDirection getDynupDirection(const std::string& direction);

struct DynupRequest {
  /* Whether the robot should stand up from the front, back or from squad */
  DynupDirection direction;
  geometry_msgs::msg::Pose l_foot_pose;  // relative to r_foot_pose
  geometry_msgs::msg::Pose r_foot_pose;
  geometry_msgs::msg::Pose l_hand_pose;
  geometry_msgs::msg::Pose r_hand_pose;
};

}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
