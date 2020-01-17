#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer() {
  ros::NodeHandle nh = ros::NodeHandle("/walking/pid_trunk");
  pid_trunk_pitch_.init(nh, false);
  reset();
}

void WalkStabilizer::reset() {
  pid_trunk_pitch_.reset();
}

WalkResponse WalkStabilizer::stabilize(const WalkResponse &response, const ros::Duration &dt) {
  // compute orientation with PID control
  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);
  // first adapt trunk pitch value based on PID controller
  double corrected_pitch = pid_trunk_pitch_.computeCommand(goal_pitch - response.current_pitch, dt);
  tf2::Quaternion corrected_orientation;
  corrected_orientation.setRPY(goal_roll, corrected_pitch, goal_yaw);

  WalkResponse stabilized_response = response;
  stabilized_response.support_foot_to_trunk.setRotation(corrected_orientation);

  return stabilized_response;
}
} // namespace bitbots_quintic_walk
