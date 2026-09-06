#include <algorithm>
#include <bitbots_head_mover/target_sampler.hpp>
#include <cmath>

namespace bitbots_head_mover {

TargetSampler::TargetSampler(const SamplerConfig& config, uint32_t seed) : config_(config), rng_(seed) {}

double TargetSampler::sampleGaussian(double center, double std, const JointLimit& limit) {
  // A non positive spread would make the normal distribution ill defined, so it
  // collapses to the center. Clamping keeps the draw reachable; it skews the
  // distribution at the edges, which is harmless for a proposal distribution.
  if (!(std > 0.0)) {
    return limit.clamp(center);
  }
  return limit.clamp(std::normal_distribution<double>(center, std)(rng_));
}

double TargetSampler::sampleUniform(const JointLimit& limit) {
  if (!(limit.upper > limit.lower)) {
    return limit.clamp(limit.lower);
  }
  return std::uniform_real_distribution<double>(limit.lower, limit.upper)(rng_);
}

std::vector<Candidate> TargetSampler::sample(const HeadPosition& current,
                                             const std::optional<HeadPosition>& last_target, const HeadLimits& limits) {
  std::vector<Candidate> candidates;
  if (config_.sample_count < 0) {
    return candidates;
  }
  candidates.reserve(static_cast<size_t>(config_.sample_count) + 2);

  // Always offer the current position. It gives the scoring something to fall
  // back on and is the candidate that keeps the head still when nothing on the
  // field is worth turning towards.
  candidates.push_back({limits.clamp(current)});

  // Offer the previous target as well, so a target that is still the best choice
  // is not lost just because no random draw happened to land on it this cycle,
  // which would otherwise let the head drift off a good target.
  if (last_target) {
    candidates.push_back({limits.clamp(*last_target)});
  }

  // The gaussian around the last target only exists once there is one; before
  // that its share is spent on the gaussian around the current position, so the
  // configured amount of local exploration is preserved either way.
  const double last_weight = last_target ? std::max(0.0, config_.last_target_weight) : 0.0;
  const double current_weight =
      std::max(0.0, config_.current_position_weight) + (last_target ? 0.0 : std::max(0.0, config_.last_target_weight));
  const double uniform_weight = std::max(0.0, config_.uniform_weight);
  const double total_weight = last_weight + current_weight + uniform_weight;

  // Without a positive weight there is no distribution to draw from; the
  // guaranteed candidates above are all that can be offered
  if (!(total_weight > 0.0)) {
    return candidates;
  }

  std::uniform_real_distribution<double> component(0.0, total_weight);
  for (int i = 0; i < config_.sample_count; i++) {
    const double pick = component(rng_);
    HeadPosition target;
    if (pick < last_weight) {
      target.yaw = sampleGaussian(last_target->yaw, config_.last_target_std, limits.yaw);
      target.pitch = sampleGaussian(last_target->pitch, config_.last_target_std, limits.pitch);
    } else if (pick < last_weight + current_weight) {
      target.yaw = sampleGaussian(current.yaw, config_.current_position_std, limits.yaw);
      target.pitch = sampleGaussian(current.pitch, config_.current_position_std, limits.pitch);
    } else {
      target.yaw = sampleUniform(limits.yaw);
      target.pitch = sampleUniform(limits.pitch);
    }
    candidates.push_back({target});
  }

  return candidates;
}

}  // namespace bitbots_head_mover
