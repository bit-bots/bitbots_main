#include <gtest/gtest.h>

#include <bitbots_head_mover/trajectory_sampler.hpp>
#include <cmath>

using bitbots_head_mover::buildCandidateTrajectory;
using bitbots_head_mover::DynamicLimits;
using bitbots_head_mover::HeadLimits;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::HeadTrajectory;
using bitbots_head_mover::HeadVelocity;
using bitbots_head_mover::isFeasible;
using bitbots_head_mover::SamplerConfig;
using bitbots_head_mover::TrajectorySampler;

namespace {
constexpr HeadLimits kLimits{{-1.23, 1.23}, {-1.23, 1.01}};

DynamicLimits makeDynamics() {
  DynamicLimits dynamics;
  dynamics.max_velocity = {4.0, 4.0};
  dynamics.max_acceleration = {14.0, 14.0};
  return dynamics;
}

SamplerConfig makeConfig() {
  SamplerConfig config;
  config.sample_count = 64;
  config.horizon = 1.0;
  config.midpoint_time = 0.5;
  config.evaluation_points = 5;
  config.feasibility_points = 21;
  return config;
}
}  // namespace

// ---------------------------------------------------------------------------
// buildCandidateTrajectory
// ---------------------------------------------------------------------------

TEST(BuildCandidateTrajectory, StartsInTheGivenState) {
  const HeadPosition start{0.1, -0.2};
  const HeadVelocity start_velocity{0.5, -0.3};
  HeadTrajectory trajectory =
      buildCandidateTrajectory(start, start_velocity, {0.3, 0.0}, {0.5, 0.2}, 0.5, 1.0, makeDynamics());

  ASSERT_TRUE(trajectory.valid());
  // Continuing the motion the head is already performing is what keeps
  // replanning every cycle from producing a jerk
  EXPECT_NEAR(trajectory.position(0.0).yaw, start.yaw, 1e-9);
  EXPECT_NEAR(trajectory.position(0.0).pitch, start.pitch, 1e-9);
  EXPECT_NEAR(trajectory.velocity(0.0).yaw, start_velocity.yaw, 1e-9);
  EXPECT_NEAR(trajectory.velocity(0.0).pitch, start_velocity.pitch, 1e-9);
}

TEST(BuildCandidateTrajectory, PassesThroughTheMidpoint) {
  const HeadPosition midpoint{0.3, 0.1};
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, midpoint, {0.5, 0.2}, 0.5, 1.0, makeDynamics());

  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.position(0.5).yaw, midpoint.yaw, 1e-9);
  EXPECT_NEAR(trajectory.position(0.5).pitch, midpoint.pitch, 1e-9);
}

TEST(BuildCandidateTrajectory, EndsAtTheEndpointAtRest) {
  const HeadPosition endpoint{0.5, 0.2};
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {0.3, 0.1}, endpoint, 0.5, 1.0, makeDynamics());

  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.position(1.0).yaw, endpoint.yaw, 1e-9);
  EXPECT_NEAR(trajectory.position(1.0).pitch, endpoint.pitch, 1e-9);
  EXPECT_NEAR(trajectory.velocity(1.0).yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.velocity(1.0).pitch, 0.0, 1e-9);
}

TEST(BuildCandidateTrajectory, MidpointVelocityFollowsTheDirectionOfTravel) {
  // Moving to a larger yaw means the head is still moving that way at the midpoint
  HeadTrajectory forward = buildCandidateTrajectory({0.0, 0.0}, {}, {0.3, 0.0}, {0.6, 0.0}, 0.5, 1.0, makeDynamics());
  ASSERT_TRUE(forward.valid());
  EXPECT_GT(forward.velocity(0.5).yaw, 0.0);

  HeadTrajectory backward = buildCandidateTrajectory({0.0, 0.0}, {}, {-0.3, 0.0}, {-0.6, 0.0}, 0.5, 1.0, makeDynamics());
  ASSERT_TRUE(backward.valid());
  EXPECT_LT(backward.velocity(0.5).yaw, 0.0);
}

TEST(BuildCandidateTrajectory, MidpointVelocityIsClampedToTheLimit) {
  DynamicLimits dynamics = makeDynamics();
  dynamics.max_velocity = {0.1, 0.1};
  // A distant endpoint would imply a midpoint velocity far above the limit
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {0.5, 0.0}, {1.0, 0.0}, 0.5, 1.0, dynamics);
  ASSERT_TRUE(trajectory.valid());
  EXPECT_LE(std::abs(trajectory.velocity(0.5).yaw), dynamics.max_velocity.yaw + 1e-9);
}

TEST(BuildCandidateTrajectory, RejectsDegenerateTimings) {
  EXPECT_FALSE(buildCandidateTrajectory({}, {}, {}, {}, 0.5, 0.0, makeDynamics()).valid());
  EXPECT_FALSE(buildCandidateTrajectory({}, {}, {}, {}, 0.0, 1.0, makeDynamics()).valid());
  // The midpoint has to sit strictly inside the horizon
  EXPECT_FALSE(buildCandidateTrajectory({}, {}, {}, {}, 1.0, 1.0, makeDynamics()).valid());
  EXPECT_FALSE(buildCandidateTrajectory({}, {}, {}, {}, 1.5, 1.0, makeDynamics()).valid());
}

// ---------------------------------------------------------------------------
// isFeasible
// ---------------------------------------------------------------------------

TEST(IsFeasible, AcceptsAGentleTrajectory) {
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {0.1, 0.05}, {0.2, 0.1}, 0.5, 1.0, makeDynamics());
  EXPECT_TRUE(isFeasible(trajectory, kLimits, makeDynamics(), 1.0, 21));
}

TEST(IsFeasible, RejectsTrajectoriesLeavingTheJointLimits) {
  // The endpoint is far outside the head's reachable range
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {1.0, 0.0}, {2.0, 0.0}, 0.5, 1.0, makeDynamics());
  EXPECT_FALSE(isFeasible(trajectory, kLimits, makeDynamics(), 1.0, 21));
}

TEST(IsFeasible, RejectsTrajectoriesExceedingTheVelocityLimit) {
  DynamicLimits dynamics = makeDynamics();
  dynamics.max_velocity = {0.05, 0.05};
  // Crossing the whole range in a second needs far more than the allowed speed
  HeadTrajectory trajectory = buildCandidateTrajectory({-1.0, 0.0}, {}, {0.0, 0.0}, {1.0, 0.0}, 0.5, 1.0, dynamics);
  EXPECT_FALSE(isFeasible(trajectory, kLimits, dynamics, 1.0, 21));
}

TEST(IsFeasible, RejectsTrajectoriesExceedingTheAccelerationLimit) {
  DynamicLimits dynamics = makeDynamics();
  dynamics.max_acceleration = {0.01, 0.01};
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {0.5, 0.0}, {1.0, 0.0}, 0.5, 1.0, dynamics);
  EXPECT_FALSE(isFeasible(trajectory, kLimits, dynamics, 1.0, 21));
}

TEST(IsFeasible, RejectsInvalidTrajectories) {
  EXPECT_FALSE(isFeasible(HeadTrajectory(), kLimits, makeDynamics(), 1.0, 21));
}

TEST(IsFeasible, NeedsAtLeastTwoCheckPoints) {
  HeadTrajectory trajectory = buildCandidateTrajectory({0.0, 0.0}, {}, {0.1, 0.0}, {0.2, 0.0}, 0.5, 1.0, makeDynamics());
  EXPECT_FALSE(isFeasible(trajectory, kLimits, makeDynamics(), 1.0, 1));
}

// ---------------------------------------------------------------------------
// TrajectorySampler
// ---------------------------------------------------------------------------

TEST(TrajectorySampler, ProducesCandidates) {
  TrajectorySampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, {}, kLimits, makeDynamics());
  EXPECT_FALSE(candidates.empty());
}

TEST(TrajectorySampler, AlwaysOffersHoldingTheCurrentPosition) {
  TrajectorySampler sampler(makeConfig());
  const HeadPosition start{0.2, -0.1};
  auto candidates = sampler.sample(start, {}, kLimits, makeDynamics());

  ASSERT_FALSE(candidates.empty());
  // The first candidate comes to rest exactly where the head already is, which
  // guarantees the scoring always has something valid to pick
  EXPECT_NEAR(candidates.front().endpoint.yaw, start.yaw, 1e-9);
  EXPECT_NEAR(candidates.front().endpoint.pitch, start.pitch, 1e-9);
}

TEST(TrajectorySampler, EveryCandidateIsFeasible) {
  TrajectorySampler sampler(makeConfig());
  const DynamicLimits dynamics = makeDynamics();
  auto candidates = sampler.sample({0.3, -0.2}, {1.0, 0.5}, kLimits, dynamics);

  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    EXPECT_TRUE(isFeasible(candidate.trajectory, kLimits, dynamics, makeConfig().horizon, makeConfig().feasibility_points));
  }
}

TEST(TrajectorySampler, EveryCandidateStartsInTheCurrentState) {
  TrajectorySampler sampler(makeConfig());
  const HeadPosition start{0.3, -0.2};
  const HeadVelocity velocity{1.0, 0.5};
  auto candidates = sampler.sample(start, velocity, kLimits, makeDynamics());

  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    EXPECT_NEAR(candidate.trajectory.position(0.0).yaw, start.yaw, 1e-9);
    EXPECT_NEAR(candidate.trajectory.velocity(0.0).yaw, velocity.yaw, 1e-9);
  }
}

TEST(TrajectorySampler, CandidatesStayWithinTheJointLimits) {
  TrajectorySampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, {}, kLimits, makeDynamics());

  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    EXPECT_TRUE(kLimits.contains(candidate.endpoint));
    EXPECT_TRUE(kLimits.contains(candidate.midpoint));
  }
}

TEST(TrajectorySampler, CandidatesAreDiverse) {
  TrajectorySampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, {}, kLimits, makeDynamics());

  ASSERT_GT(candidates.size(), 2u);
  double min_yaw = 1e9, max_yaw = -1e9;
  for (const auto& candidate : candidates) {
    min_yaw = std::min(min_yaw, candidate.endpoint.yaw);
    max_yaw = std::max(max_yaw, candidate.endpoint.yaw);
  }
  // A sampler that always returned the same candidate would make the whole
  // search pointless
  EXPECT_GT(max_yaw - min_yaw, 0.1);
}

TEST(TrajectorySampler, IsDeterministicForAGivenSeed) {
  TrajectorySampler first(makeConfig(), 1234);
  TrajectorySampler second(makeConfig(), 1234);
  auto a = first.sample({0.0, 0.0}, {}, kLimits, makeDynamics());
  auto b = second.sample({0.0, 0.0}, {}, kLimits, makeDynamics());

  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++) {
    EXPECT_DOUBLE_EQ(a[i].endpoint.yaw, b[i].endpoint.yaw);
    EXPECT_DOUBLE_EQ(a[i].endpoint.pitch, b[i].endpoint.pitch);
  }
}

TEST(TrajectorySampler, DifferentSeedsExploreDifferently) {
  TrajectorySampler first(makeConfig(), 1);
  TrajectorySampler second(makeConfig(), 2);
  auto a = first.sample({0.0, 0.0}, {}, kLimits, makeDynamics());
  auto b = second.sample({0.0, 0.0}, {}, kLimits, makeDynamics());

  ASSERT_FALSE(a.empty());
  ASSERT_FALSE(b.empty());
  // The hold candidate is identical by construction, so compare a random one
  ASSERT_GT(a.size(), 1u);
  ASSERT_GT(b.size(), 1u);
  EXPECT_NE(a[1].endpoint.yaw, b[1].endpoint.yaw);
}

TEST(TrajectorySampler, SlowJointsRestrictHowFarCandidatesReach) {
  DynamicLimits dynamics = makeDynamics();
  dynamics.max_velocity = {0.2, 0.2};
  TrajectorySampler sampler(makeConfig());
  auto candidates = sampler.sample({0.0, 0.0}, {}, kLimits, dynamics);

  ASSERT_FALSE(candidates.empty());
  for (const auto& candidate : candidates) {
    // Nothing beyond what the joint could travel within the horizon
    EXPECT_LE(std::abs(candidate.endpoint.yaw), 0.2 * makeConfig().horizon + 1e-9);
  }
}

TEST(TrajectorySampler, ZeroSampleCountYieldsNothing) {
  SamplerConfig config = makeConfig();
  config.sample_count = 0;
  TrajectorySampler sampler(config);
  EXPECT_TRUE(sampler.sample({0.0, 0.0}, {}, kLimits, makeDynamics()).empty());
}
