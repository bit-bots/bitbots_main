#pragma once

#include <bitbots_head_mover/head_trajectory.hpp>
#include <bitbots_head_mover/types.hpp>
#include <random>
#include <vector>

/// Random generation of candidate head trajectories.
namespace bitbots_head_mover {

/// The dynamic limits a candidate trajectory has to respect.
struct DynamicLimits {
  HeadVelocity max_velocity{4.0, 4.0};
  HeadAcceleration max_acceleration{14.0, 14.0};
};

/// Shape and number of the sampled candidates.
struct SamplerConfig {
  /// How many candidates are drawn per planning cycle.
  int sample_count = 64;
  /// How far into the future a candidate reaches.
  double horizon = 1.0;
  /// When the intermediate waypoint sits. Splitting the horizon lets a candidate
  /// curve instead of only moving straight towards its endpoint.
  double midpoint_time = 0.5;
  /// How many points along a candidate are evaluated by the scoring.
  ///
  /// The points are spread evenly over the horizon and always end at the
  /// endpoint, never including the start: all candidates share the same start,
  /// so scoring it cannot tell them apart. Two points therefore evaluate the
  /// midpoint and the goal point.
  int evaluation_points = 2;
  /// How many points are checked when verifying the dynamic limits. This is
  /// finer than the scoring, because a limit violation between two scoring
  /// points would go unnoticed otherwise.
  int feasibility_points = 21;
  /// How many times a single candidate is redrawn before giving up on it.
  int max_attempts_per_sample = 8;
  /// How far the intermediate waypoint may deviate from the straight path to the
  /// endpoint, in radians. This is what lets a candidate curve; drawing the
  /// midpoint independently instead would mostly produce detours that violate
  /// the acceleration limit and get rejected.
  double midpoint_deviation = 0.3;
};

/// A candidate trajectory together with the state it ends in.
struct Candidate {
  HeadTrajectory trajectory;
  /// The head position the candidate ends at, kept for debug output.
  HeadPosition endpoint;
  /// The intermediate waypoint the candidate was drawn with.
  HeadPosition midpoint;
};

/// Build a candidate trajectory through a midpoint to a resting endpoint.
///
/// The trajectory starts in the given state, so a candidate always continues the
/// motion the head is already performing instead of assuming it stands still.
/// The midpoint velocity is derived rather than sampled: it follows the overall
/// direction of travel, which keeps the trajectory from having to stop and
/// restart in the middle. The endpoint is reached at rest, so that a candidate
/// that is never replaced still leaves the head in a defined state.
HeadTrajectory buildCandidateTrajectory(const HeadPosition& start, const HeadVelocity& start_velocity,
                                        const HeadPosition& midpoint, const HeadPosition& endpoint, double midpoint_time,
                                        double horizon, const DynamicLimits& dynamics);

/// Whether a trajectory stays inside the joint and dynamic limits.
///
/// Checks position, velocity and acceleration at evenly spaced points, which is
/// an approximation, but a quintic between two waypoints has no room to hide a
/// violation between sufficiently dense samples.
bool isFeasible(const HeadTrajectory& trajectory, const HeadLimits& limits, const DynamicLimits& dynamics,
                double horizon, int feasibility_points);

/// Draws candidate head trajectories.
///
/// Sampling is done by rejection: candidates are drawn from a range that is
/// roughly reachable within the horizon and then verified against the actual
/// limits, so no candidate that the head cannot physically follow is ever
/// scored. The candidate set always contains the trajectory that holds the
/// current position, which guarantees that there is something to choose from
/// even when every random draw is rejected.
class TrajectorySampler {
 public:
  explicit TrajectorySampler(const SamplerConfig& config = {}, uint32_t seed = 42);

  void setConfig(const SamplerConfig& config) { config_ = config; }
  const SamplerConfig& config() const { return config_; }

  /// Draw candidates starting from the given head state.
  std::vector<Candidate> sample(const HeadPosition& start, const HeadVelocity& start_velocity, const HeadLimits& limits,
                                const DynamicLimits& dynamics);

 private:
  /// Draw a single joint value within the reachable part of its limits.
  double sampleJoint(double start, double max_velocity, double duration, const JointLimit& limit);

  /// Draw a single joint value near a given center, staying inside the limits.
  double sampleAround(double center, double deviation, const JointLimit& limit);

  SamplerConfig config_;
  std::mt19937 rng_;
};

}  // namespace bitbots_head_mover
