#include <algorithm>
#include <bitbots_head_mover/trajectory_sampler.hpp>
#include <cmath>

namespace bitbots_head_mover {

HeadTrajectory buildCandidateTrajectory(const HeadPosition& start, const HeadVelocity& start_velocity,
                                        const HeadPosition& midpoint, const HeadPosition& endpoint,
                                        double midpoint_time, double horizon, const DynamicLimits& dynamics) {
  HeadTrajectory trajectory;
  if (!(horizon > 0.0) || !(midpoint_time > 0.0) || midpoint_time >= horizon) {
    return trajectory;
  }

  // The midpoint velocity follows the overall direction of travel, scaled by the
  // total duration. This is the usual finite difference tangent: it keeps the
  // trajectory moving through the midpoint instead of stopping there, without
  // adding another sampled dimension. Clamping it to the maximum keeps the
  // waypoint itself from being the reason a candidate is infeasible.
  HeadVelocity midpoint_velocity{
      std::clamp((endpoint.yaw - start.yaw) / horizon, -dynamics.max_velocity.yaw, dynamics.max_velocity.yaw),
      std::clamp((endpoint.pitch - start.pitch) / horizon, -dynamics.max_velocity.pitch, dynamics.max_velocity.pitch)};

  trajectory.addPoint(0.0, start, start_velocity);
  trajectory.addPoint(midpoint_time, midpoint, midpoint_velocity);
  // The endpoint is reached at rest, so a candidate that is never replaced
  // leaves the head in a defined state rather than drifting on
  trajectory.addPoint(horizon, endpoint);
  trajectory.finalize();
  return trajectory;
}

bool isFeasible(const HeadTrajectory& trajectory, const HeadLimits& limits, const DynamicLimits& dynamics,
                double horizon, int feasibility_points) {
  if (!trajectory.valid() || feasibility_points < 2 || !(horizon > 0.0)) {
    return false;
  }

  for (int i = 0; i < feasibility_points; i++) {
    const double t = horizon * static_cast<double>(i) / static_cast<double>(feasibility_points - 1);

    const HeadPosition position = trajectory.position(t);
    if (!limits.contains(position)) {
      return false;
    }

    const HeadVelocity velocity = trajectory.velocity(t);
    if (std::abs(velocity.yaw) > dynamics.max_velocity.yaw ||
        std::abs(velocity.pitch) > dynamics.max_velocity.pitch) {
      return false;
    }

    const HeadAcceleration acceleration = trajectory.acceleration(t);
    if (std::abs(acceleration.yaw) > dynamics.max_acceleration.yaw ||
        std::abs(acceleration.pitch) > dynamics.max_acceleration.pitch) {
      return false;
    }
  }

  return true;
}

TrajectorySampler::TrajectorySampler(const SamplerConfig& config, uint32_t seed) : config_(config), rng_(seed) {}

double TrajectorySampler::sampleJoint(double start, double max_velocity, double duration, const JointLimit& limit) {
  // Only draw from the part of the joint range that can plausibly be reached in
  // the available time. Drawing from the full range would make almost every
  // candidate fail the feasibility check when the horizon is short.
  const double reach = std::abs(max_velocity) * duration;
  const double lower = std::max(limit.lower, start - reach);
  const double upper = std::min(limit.upper, start + reach);
  if (!(upper > lower)) {
    return std::clamp(start, limit.lower, limit.upper);
  }
  return std::uniform_real_distribution<double>(lower, upper)(rng_);
}

double TrajectorySampler::sampleAround(double center, double deviation, const JointLimit& limit) {
  const double lower = std::max(limit.lower, center - std::abs(deviation));
  const double upper = std::min(limit.upper, center + std::abs(deviation));
  if (!(upper > lower)) {
    return std::clamp(center, limit.lower, limit.upper);
  }
  return std::uniform_real_distribution<double>(lower, upper)(rng_);
}

std::vector<Candidate> TrajectorySampler::sample(const HeadPosition& start, const HeadVelocity& start_velocity,
                                                 const HeadLimits& limits, const DynamicLimits& dynamics) {
  std::vector<Candidate> candidates;
  if (config_.sample_count <= 0) {
    return candidates;
  }
  candidates.reserve(static_cast<size_t>(config_.sample_count) + 1);

  // Always offer the candidate that comes to rest where the head already is. It
  // gives the scoring something to fall back on when every random draw is
  // rejected, and it is the trajectory that suppresses motion when nothing on
  // the field is worth looking at.
  {
    const HeadPosition hold = limits.clamp(start);
    HeadTrajectory trajectory =
        buildCandidateTrajectory(start, start_velocity, hold, hold, config_.midpoint_time, config_.horizon, dynamics);
    if (isFeasible(trajectory, limits, dynamics, config_.horizon, config_.feasibility_points)) {
      candidates.push_back({std::move(trajectory), hold, hold});
    }
  }

  // Where the midpoint would sit if the head travelled straight to the endpoint
  const double straight_fraction = config_.midpoint_time / config_.horizon;

  for (int i = 0; i < config_.sample_count; i++) {
    for (int attempt = 0; attempt < config_.max_attempts_per_sample; attempt++) {
      const HeadPosition endpoint{
          sampleJoint(start.yaw, dynamics.max_velocity.yaw, config_.horizon, limits.yaw),
          sampleJoint(start.pitch, dynamics.max_velocity.pitch, config_.horizon, limits.pitch)};

      // Draw the midpoint around the straight path towards the endpoint rather
      // than independently of it. An independent midpoint usually demands a
      // detour that the acceleration limit cannot serve, which is why doing it
      // that way rejected roughly a third of all draws. The deviation still
      // allows curved sweeps that cover more ground than a straight move.
      const HeadPosition midpoint{
          sampleAround(start.yaw + (endpoint.yaw - start.yaw) * straight_fraction, config_.midpoint_deviation,
                       limits.yaw),
          sampleAround(start.pitch + (endpoint.pitch - start.pitch) * straight_fraction, config_.midpoint_deviation,
                       limits.pitch)};

      HeadTrajectory trajectory = buildCandidateTrajectory(start, start_velocity, midpoint, endpoint,
                                                           config_.midpoint_time, config_.horizon, dynamics);
      if (isFeasible(trajectory, limits, dynamics, config_.horizon, config_.feasibility_points)) {
        candidates.push_back({std::move(trajectory), endpoint, midpoint});
        break;
      }
    }
  }

  return candidates;
}

}  // namespace bitbots_head_mover
