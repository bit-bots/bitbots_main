#pragma once

#include <algorithm>

/// Shared value types for the head mover.
///
/// These are intentionally dependency-free so that every component built on top
/// of them (search patterns, trajectories, sampling and scoring) can be unit
/// tested without a ROS node, a TF buffer or a parameter listener.
namespace bitbots_head_mover {

/// A head configuration in joint space.
///
/// Both values are in radians unless a function explicitly documents that it
/// works on degrees (the search pattern parameters are configured in degrees).
struct HeadPosition {
  double yaw = 0.0;
  double pitch = 0.0;
};

/// A head velocity in joint space, in radians per second.
struct HeadVelocity {
  double yaw = 0.0;
  double pitch = 0.0;
};

/// A head acceleration in joint space, in radians per second squared.
struct HeadAcceleration {
  double yaw = 0.0;
  double pitch = 0.0;
};

/// The inclusive bounds of a single joint.
struct JointLimit {
  double lower = 0.0;
  double upper = 0.0;

  /// Clamp a value into the bounds.
  constexpr double clamp(double value) const { return std::clamp(value, lower, upper); }

  /// Whether a value lies strictly inside the bounds.
  ///
  /// The bounds are treated as exclusive because the head mover has always
  /// rejected goals that sit exactly on a limit as a collision.
  constexpr bool contains(double value) const { return lower < value && value < upper; }
};

/// The joint limits of both head joints.
struct HeadLimits {
  JointLimit yaw;
  JointLimit pitch;

  /// Clamp a head position into the limits.
  constexpr HeadPosition clamp(HeadPosition position) const {
    return {yaw.clamp(position.yaw), pitch.clamp(position.pitch)};
  }

  /// Whether a head position lies strictly inside the limits of both joints.
  constexpr bool contains(const HeadPosition& position) const {
    return yaw.contains(position.yaw) && pitch.contains(position.pitch);
  }
};

}  // namespace bitbots_head_mover
