#pragma once

#include <bitbots_head_mover/types.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <vector>

/// Time parameterized head trajectories in joint space.
namespace bitbots_head_mover {

/// A pair of quintic splines describing the yaw and pitch joint over time.
///
/// This is a thin wrapper around two bitbots_splines::SmoothSpline instances
/// that keeps both joints in lockstep, so a trajectory can be evaluated as a
/// single head position instead of two independent scalars.
class HeadTrajectory {
 public:
  /// Append a waypoint. Waypoints must be added with increasing time.
  void addPoint(double time, const HeadPosition& position, const HeadVelocity& velocity = {},
                const HeadAcceleration& acceleration = {});

  /// Compute the spline interpolation over the added waypoints.
  ///
  /// Must be called after the last waypoint was added and before the trajectory
  /// is evaluated. A trajectory with fewer than two waypoints stays invalid.
  void finalize();

  /// Whether the trajectory was finalized and can be evaluated.
  bool valid() const { return valid_; }

  /// The time of the last waypoint, or zero for an unfinalized trajectory.
  double duration() const { return duration_; }

  /// The number of waypoints that were added.
  size_t size() const { return size_; }

  /// The head position at the given time.
  HeadPosition position(double time) const;

  /// The head velocity at the given time.
  HeadVelocity velocity(double time) const;

  /// The head acceleration at the given time.
  HeadAcceleration acceleration(double time) const;

 private:
  bitbots_splines::SmoothSpline yaw_;
  bitbots_splines::SmoothSpline pitch_;
  double duration_ = 0.0;
  size_t size_ = 0;
  bool valid_ = false;
};

/// A looping search pattern trajectory.
///
/// The trajectory starts at the current head position and smoothly transitions
/// into the pattern, so switching head modes does not result in an abrupt
/// movement. Only the part after the transition loops; the transition itself is
/// played exactly once.
struct SearchPatternTrajectory {
  HeadTrajectory trajectory;
  /// Duration of the one-off segment leading from the start position into the pattern.
  double transition_duration = 0.0;
  /// Duration of one full pattern cycle, excluding the transition.
  double cycle_duration = 0.0;

  /// Whether the trajectory can be evaluated.
  bool valid() const { return trajectory.valid() && cycle_duration > 0.0; }

  /// Map a time elapsed since the trajectory started to a time on the trajectory.
  ///
  /// Times inside the transition are passed through, later times wrap around the
  /// cyclic part of the trajectory.
  double phase(double elapsed) const;
};

/// Build a looping trajectory from search pattern keyframes.
///
/// Note the mixed units: the keyframes are given in degrees, because that is how
/// the search patterns are configured, while start and the resulting trajectory
/// are in radians, because that is what the joints use. Waypoint
/// timestamps are distributed proportionally to the Euclidean distance between
/// consecutive keyframes, so the cycle takes exactly cycle_time seconds and the
/// head moves at a roughly constant angular speed. The loop is closed by
/// appending the first keyframe at the end. The duration of the prepended
/// transition from start follows from its distance and transition_speed.
SearchPatternTrajectory buildSearchPatternTrajectory(const std::vector<HeadPosition>& pattern_deg, double cycle_time,
                                                     const HeadPosition& start, double transition_speed);

/// Slow down the joint that has to travel less so both joints arrive together.
///
/// Returns the speed the slower-traveling joint should use, or zero if the
/// faster joint does not have to move at all.
double calculateLowerSpeed(double delta_faster_joint, double delta_joint, double speed);

/// Adjust the maximum joint speeds so both joints reach the goal at the same time.
///
/// The joint that has to travel the shorter distance is slowed down to match the
/// travel time of the other one. Speeds are never increased beyond max_speeds.
HeadVelocity adjustSpeeds(const HeadPosition& goal, const HeadPosition& current, const HeadVelocity& max_speeds);

}  // namespace bitbots_head_mover
