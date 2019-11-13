#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_

struct DynupResponse {
  geometry_msgs::Point support_point;
  geometry_msgs::PoseStamped l_foot_goal_pose;
  geometry_msgs::PoseStamped trunk_goal_pose;
};

struct DynupRequest {
  /* Whether the robot should stand up from the front or back */
  bool front;
  geometry_msgs::Pose l_foot_pose;
  geometry_msgs::Pose trunk_pose;
  geometry_msgs::Pose l_hand_pose;
};

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
