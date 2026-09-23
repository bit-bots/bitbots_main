#pragma once

#include <bitbots_head_mover/types.hpp>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

/// Inverse kinematics for pointing the head at a single point.
namespace bitbots_head_mover {

/// The angle between the head pitch joint and the camera's optical axis.
///
/// The camera is mounted tilted forward relative to the head pitch link, so a
/// pitch goal computed from a point has to be corrected by this angle to make
/// the camera, rather than the link, point at the target.
constexpr double kCameraPitchOffset = M_PI / 180.0 * 30.0;

/// Compute the head joint goals that make the camera look at a point.
///
/// The point has to be given twice, expressed in the head yaw link and in the
/// head pitch link, because each joint is solved in its own frame. The result is
/// relative to the current head position, which therefore has to be passed in.
HeadPosition motorGoalsFromPoint(const geometry_msgs::msg::Point& head_yaw_point,
                                 const geometry_msgs::msg::Point& head_pitch_point, const HeadPosition& current,
                                 double camera_pitch_offset = kCameraPitchOffset);

}  // namespace bitbots_head_mover
