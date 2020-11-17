#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_

#include <tf2/LinearMath/Transform.h>

struct DynupResponse {
  tf2::Transform l_foot_goal_pose;
  tf2::Transform r_foot_goal_pose;
  tf2::Transform l_hand_goal_pose;
  tf2::Transform r_hand_goal_pose;
};

struct DynupRequest {
  /* Whether the robot should stand up from the front, back or from squad */
  std::string direction;
  geometry_msgs::Pose l_foot_pose;
  geometry_msgs::Pose r_foot_pose;
  geometry_msgs::Pose l_hand_pose;
  geometry_msgs::Pose r_hand_pose;
};

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
