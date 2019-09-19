#ifndef BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
#define BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace bitbots_quintic_walk {

struct WalkRequest {
  tf2::Transform orders;
  bool walkable_state;
};

struct WalkResponse {
  tf2::Transform support_foot_to_flying_foot;
  tf2::Transform support_foot_to_trunk;
  bool is_double_support;
  bool is_left_support_foot;
};

}

#endif //BITBOTS_MOTION_BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_UTILS_H_
