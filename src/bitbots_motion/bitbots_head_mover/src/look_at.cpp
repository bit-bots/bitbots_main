#include <bitbots_head_mover/look_at.hpp>
#include <cmath>

namespace bitbots_head_mover {

HeadPosition motorGoalsFromPoint(const geometry_msgs::msg::Point& head_yaw_point,
                                 const geometry_msgs::msg::Point& head_pitch_point, const HeadPosition& current,
                                 double camera_pitch_offset) {
  // The yaw joint only has to turn towards the point's direction in the ground plane
  double rel_head_yaw = std::atan2(head_yaw_point.y, head_yaw_point.x);

  // The pitch joint has to tilt by the point's elevation over the joint's plane
  double rel_head_pitch = -std::atan2(head_pitch_point.z, std::sqrt(head_pitch_point.x * head_pitch_point.x +
                                                                    head_pitch_point.y * head_pitch_point.y));

  // Both angles are relative to the current head position
  return {rel_head_yaw + current.yaw, rel_head_pitch + (current.pitch - camera_pitch_offset)};
}

}  // namespace bitbots_head_mover
