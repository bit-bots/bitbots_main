#include <gtest/gtest.h>

#include <algorithm>
#include <bitbots_head_mover/target_sampler.hpp>
#include <cmath>
#include <optional>

using bitbots_head_mover::Candidate;
using bitbots_head_mover::HeadLimits;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::SamplerConfig;
using bitbots_head_mover::TargetSampler;

namespace {
constexpr HeadLimits kLimits{{-1.23, 1.23}, {-1.23, 1.01}};

SamplerConfig makeConfig() {
  SamplerConfig config;
  config.sample_count = 64;
  config.last_target_weight = 0.4;
  config.current_position_weight = 0.3;
  config.uniform_weight = 0.3;
  config.last_target_std = 0.3;
  config.current_position_std = 0.5;
  return config;
}

/// Whether a position lies within the inclusive joint bounds.
bool withinLimits(const HeadPosition& position, const HeadLimits& limits) {
  return position.yaw >= limits.yaw.lower && position.yaw <= limits.yaw.upper && position.pitch >= limits.pitch.lower &&
         position.pitch <= limits.pitch.upper;
}
}  // namespace

// ---------------------------------------------------------------------------
// TargetSampler
// ---------------------------------------------------------------------------

TEST(TargetSampler, ProducesCandidates) {
  TargetSampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, std::nullopt, kLimits);
  EXPECT_FALSE(candidates.empty());
}

TEST(TargetSampler, AlwaysOffersTheCurrentPosition) {
  TargetSampler sampler(makeConfig());
  const HeadPosition current{0.2, -0.1};
  auto candidates = sampler.sample(current, std::nullopt, kLimits);

  ASSERT_FALSE(candidates.empty());
  // The first candidate is exactly where the head already is, which guarantees
  // the scoring always has something valid to pick and a way to hold still
  EXPECT_NEAR(candidates.front().target.yaw, current.yaw, 1e-9);
  EXPECT_NEAR(candidates.front().target.pitch, current.pitch, 1e-9);
}

TEST(TargetSampler, OffersThePreviousTargetWhenGiven) {
  TargetSampler sampler(makeConfig());
  const HeadPosition last{0.7, 0.3};
  auto candidates = sampler.sample({0.0, 0.0}, last, kLimits);

  // The previous target has to be among the candidates so a still-best target is
  // not lost to sampling noise between cycles
  const bool found = std::any_of(candidates.begin(), candidates.end(), [&](const Candidate& candidate) {
    return std::abs(candidate.target.yaw - last.yaw) < 1e-9 && std::abs(candidate.target.pitch - last.pitch) < 1e-9;
  });
  EXPECT_TRUE(found);
}

TEST(TargetSampler, CandidatesStayWithinTheJointLimits) {
  TargetSampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, HeadPosition{1.0, 0.9}, kLimits);

  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    EXPECT_TRUE(withinLimits(candidate.target, kLimits));
  }
}

TEST(TargetSampler, CandidatesAreDiverse) {
  TargetSampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, std::nullopt, kLimits);

  ASSERT_GT(candidates.size(), 2u);
  double min_yaw = 1e9, max_yaw = -1e9;
  for (const auto& candidate : candidates) {
    min_yaw = std::min(min_yaw, candidate.target.yaw);
    max_yaw = std::max(max_yaw, candidate.target.yaw);
  }
  // A sampler that always returned the same target would make the search pointless
  EXPECT_GT(max_yaw - min_yaw, 0.1);
}

TEST(TargetSampler, ConcentratesAroundTheLastTarget) {
  // With almost all weight on the last target and a tight spread, the bulk of
  // the draws should cluster around it
  SamplerConfig config = makeConfig();
  config.last_target_weight = 1.0;
  config.current_position_weight = 0.0;
  config.uniform_weight = 0.0;
  config.last_target_std = 0.1;
  config.sample_count = 200;

  TargetSampler sampler(config);
  const HeadPosition last{0.6, 0.2};
  auto candidates = sampler.sample({-1.0, -1.0}, last, kLimits);

  int near = 0;
  for (const auto& candidate : candidates) {
    if (std::hypot(candidate.target.yaw - last.yaw, candidate.target.pitch - last.pitch) < 0.3) {
      near++;
    }
  }
  // The great majority of the drawn samples should sit close to the last target
  EXPECT_GT(near, static_cast<int>(config.sample_count) / 2);
}

TEST(TargetSampler, IsDeterministicForAGivenSeed) {
  TargetSampler first(makeConfig(), 1234);
  TargetSampler second(makeConfig(), 1234);
  auto a = first.sample({0.0, 0.0}, HeadPosition{0.3, 0.1}, kLimits);
  auto b = second.sample({0.0, 0.0}, HeadPosition{0.3, 0.1}, kLimits);

  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++) {
    EXPECT_DOUBLE_EQ(a[i].target.yaw, b[i].target.yaw);
    EXPECT_DOUBLE_EQ(a[i].target.pitch, b[i].target.pitch);
  }
}

TEST(TargetSampler, DifferentSeedsExploreDifferently) {
  TargetSampler first(makeConfig(), 1);
  TargetSampler second(makeConfig(), 2);
  auto a = first.sample({0.0, 0.0}, std::nullopt, kLimits);
  auto b = second.sample({0.0, 0.0}, std::nullopt, kLimits);

  ASSERT_GT(a.size(), 1u);
  ASSERT_GT(b.size(), 1u);
  // The current position candidate is identical by construction, so compare a
  // randomly drawn one
  EXPECT_NE(a[1].target.yaw, b[1].target.yaw);
}

TEST(TargetSampler, ZeroSampleCountStillOffersTheGuaranteedCandidates) {
  SamplerConfig config = makeConfig();
  config.sample_count = 0;
  TargetSampler sampler(config);
  const HeadPosition current{0.2, -0.1};
  auto candidates = sampler.sample(current, HeadPosition{0.5, 0.3}, kLimits);

  // No random draws, but the current position and the previous target are always
  // offered so the scoring never runs out of options
  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_NEAR(candidates.front().target.yaw, current.yaw, 1e-9);
}

TEST(TargetSampler, NegativeSampleCountYieldsNothing) {
  SamplerConfig config = makeConfig();
  config.sample_count = -1;
  TargetSampler sampler(config);
  EXPECT_TRUE(sampler.sample({0.0, 0.0}, std::nullopt, kLimits).empty());
}

TEST(TargetSampler, WithoutPositiveWeightsOnlyTheGuaranteedCandidatesRemain) {
  SamplerConfig config = makeConfig();
  config.last_target_weight = 0.0;
  config.current_position_weight = 0.0;
  config.uniform_weight = 0.0;
  TargetSampler sampler(config);

  // No component to draw from, so only the current position is offered
  auto candidates = sampler.sample({0.1, 0.1}, std::nullopt, kLimits);
  EXPECT_EQ(candidates.size(), 1u);
}
