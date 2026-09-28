#pragma once

#include <bitbots_head_mover/types.hpp>
#include <optional>
#include <random>
#include <vector>

/// Random generation of candidate head targets.
namespace bitbots_head_mover {

/// Shape and number of the sampled candidate targets.
///
/// Targets are drawn from a mixture of three components: a gaussian around the
/// previously selected target, a gaussian around the current head position and a
/// uniform base distribution over the whole reachable range. The two gaussians
/// concentrate the search where the head already is or was heading, while the
/// uniform floor keeps it exploring the rest of the field so a better target is
/// eventually found.
struct SamplerConfig {
  /// How many targets are drawn per planning cycle.
  int sample_count = 64;

  /// Relative share of the samples drawn around the previously selected target.
  ///
  /// The three weights do not have to sum to one; they are normalized. At least
  /// one of them has to be positive, otherwise there is no distribution to draw
  /// from. When there is no previous target yet, its share is folded into the
  /// gaussian around the current position.
  double last_target_weight = 0.4;
  /// Relative share of the samples drawn around the current head position.
  double current_position_weight = 0.3;
  /// Relative share of the samples drawn uniformly over the reachable range.
  double uniform_weight = 0.3;

  /// Standard deviation, in radians, of the gaussian around the last target.
  double last_target_std = 0.3;
  /// Standard deviation, in radians, of the gaussian around the current position.
  double current_position_std = 0.5;
};

/// A candidate target head position, kept for scoring and debug output.
struct Candidate {
  /// The head position the candidate would look at.
  HeadPosition target;
};

/// Draws candidate target head positions.
///
/// Every draw is clamped into the joint limits, so no candidate ever asks the
/// head to leave its reachable range. The candidate set always contains the
/// current head position, which guarantees that there is something to choose from
/// and gives the scoring a way to keep the head still when nothing is worth
/// looking at, and the previous target when one exists, so a still-best target is
/// not lost to sampling noise between cycles.
class TargetSampler {
 public:
  explicit TargetSampler(const SamplerConfig& config = {}, uint32_t seed = 42);

  void setConfig(const SamplerConfig& config) { config_ = config; }
  const SamplerConfig& config() const { return config_; }

  /// Draw candidate targets around the current position and the last target.
  ///
  /// last_target is empty on the first cycle, which folds its mixture share into
  /// the gaussian around the current position.
  std::vector<Candidate> sample(const HeadPosition& current, const std::optional<HeadPosition>& last_target,
                                const HeadLimits& limits);

 private:
  /// Draw a single joint value from a gaussian around a center, clamped to the limit.
  double sampleGaussian(double center, double std, const JointLimit& limit);
  /// Draw a single joint value uniformly over the limit.
  double sampleUniform(const JointLimit& limit);

  SamplerConfig config_;
  std::mt19937 rng_;
};

}  // namespace bitbots_head_mover
