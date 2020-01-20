#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_

#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>

struct DynupResponse {
  geometry_msgs::Point support_point;
  tf2::Transform l_foot_pose;
  tf2::Transform trunk_pose;
};

struct DynupRequest {
  /* Whether the robot should stand up from the front or back */
  bool front;
  geometry_msgs::Pose l_foot_pose;
  geometry_msgs::Pose trunk_pose;
};

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_UTILS_H_
