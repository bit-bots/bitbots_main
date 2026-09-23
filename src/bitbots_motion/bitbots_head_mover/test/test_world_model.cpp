#include <gtest/gtest.h>

#include <bitbots_head_mover/world_model.hpp>

using bitbots_head_mover::TimedTarget;
using bitbots_head_mover::weightFromCovariance;
using bitbots_head_mover::WorldModel;
using bitbots_head_mover::WorldModelConfig;

namespace {
WorldModelConfig makeConfig() {
  WorldModelConfig config;
  config.filtered_ball_timeout = 5.0;
  config.raw_ball_timeout = 0.5;
  config.team_ball_timeout = 3.0;
  config.robot_timeout = 1.0;
  config.covariance_half_weight = 0.5;
  return config;
}
}  // namespace

// ---------------------------------------------------------------------------
// Covariance weighting
// ---------------------------------------------------------------------------

TEST(WeightFromCovariance, PerfectEstimateKeepsFullWeight) { EXPECT_DOUBLE_EQ(weightFromCovariance(0.0, 0.5), 1.0); }

TEST(WeightFromCovariance, HalvesAtTheConfiguredScale) { EXPECT_DOUBLE_EQ(weightFromCovariance(0.5, 0.5), 0.5); }

TEST(WeightFromCovariance, DecreasesMonotonically) {
  EXPECT_GT(weightFromCovariance(0.1, 0.5), weightFromCovariance(1.0, 0.5));
  EXPECT_GT(weightFromCovariance(1.0, 0.5), weightFromCovariance(10.0, 0.5));
}

TEST(WeightFromCovariance, NeverReachesZero) {
  // A very uncertain ball is still better than no ball at all
  EXPECT_GT(weightFromCovariance(1000.0, 0.5), 0.0);
}

TEST(WeightFromCovariance, StaysWithinTheUnitInterval) {
  for (double covariance = 0.0; covariance < 20.0; covariance += 0.25) {
    const double weight = weightFromCovariance(covariance, 0.5);
    EXPECT_GT(weight, 0.0);
    EXPECT_LE(weight, 1.0);
  }
}

TEST(WeightFromCovariance, TreatsNegativeCovarianceAsPerfect) {
  // A negative variance is meaningless, but must not produce a weight above one
  EXPECT_DOUBLE_EQ(weightFromCovariance(-1.0, 0.5), 1.0);
}

// ---------------------------------------------------------------------------
// Filtered ball
// ---------------------------------------------------------------------------

TEST(WorldModel, StartsEmpty) {
  WorldModel world(makeConfig());
  EXPECT_FALSE(world.filteredBall().has_value());
  EXPECT_TRUE(world.rawBalls().empty());
  EXPECT_TRUE(world.robots().empty());
  EXPECT_TRUE(world.teamBalls().empty());
  EXPECT_FALSE(world.hasAnyBall());
}

TEST(WorldModel, StoresTheFilteredBallWithACovarianceWeight) {
  WorldModel world(makeConfig());
  world.setFilteredBall({1.0, 2.0, 0.0}, 0.5, 10.0);

  ASSERT_TRUE(world.filteredBall().has_value());
  EXPECT_DOUBLE_EQ(world.filteredBall()->position.x(), 1.0);
  EXPECT_DOUBLE_EQ(world.filteredBall()->weight, 0.5);
  EXPECT_TRUE(world.hasAnyBall());
}

TEST(WorldModel, DropsTheFilteredBallAfterItsTimeout) {
  WorldModel world(makeConfig());
  world.setFilteredBall({1.0, 2.0, 0.0}, 0.1, 10.0);

  world.prune(14.9);
  EXPECT_TRUE(world.filteredBall().has_value());

  world.prune(15.1);
  EXPECT_FALSE(world.filteredBall().has_value());
}

// ---------------------------------------------------------------------------
// Raw detections
// ---------------------------------------------------------------------------

TEST(WorldModel, RawBallsExpireIndividually) {
  WorldModel world(makeConfig());
  world.setRawBalls({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}, TimedTarget{{2.0, 0.0, 0.0}, 1.0, 10.4}});

  // The older detection is past the short raw timeout, the newer one is not
  world.prune(10.6);
  ASSERT_EQ(world.rawBalls().size(), 1u);
  EXPECT_DOUBLE_EQ(world.rawBalls().front().position.x(), 2.0);
}

TEST(WorldModel, RawBallsUseAShorterTimeoutThanTheFilteredEstimate) {
  WorldModel world(makeConfig());
  world.setFilteredBall({1.0, 0.0, 0.0}, 0.1, 10.0);
  world.setRawBalls({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});

  world.prune(11.0);
  EXPECT_TRUE(world.rawBalls().empty());
  EXPECT_TRUE(world.filteredBall().has_value());
}

TEST(WorldModel, RobotsExpireAfterTheirTimeout) {
  WorldModel world(makeConfig());
  world.setRobots({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});

  world.prune(10.9);
  EXPECT_EQ(world.robots().size(), 1u);

  world.prune(11.1);
  EXPECT_TRUE(world.robots().empty());
}

TEST(WorldModel, SettingDetectionsReplacesThePreviousOnes) {
  WorldModel world(makeConfig());
  world.setRawBalls({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});
  world.setRawBalls({TimedTarget{{2.0, 0.0, 0.0}, 1.0, 10.1}});

  ASSERT_EQ(world.rawBalls().size(), 1u);
  EXPECT_DOUBLE_EQ(world.rawBalls().front().position.x(), 2.0);
}

// ---------------------------------------------------------------------------
// Team balls
// ---------------------------------------------------------------------------

TEST(WorldModel, TracksTeamBallsPerRobot) {
  WorldModel world(makeConfig());
  world.setTeamBall(2, {1.0, 0.0, 0.0}, 0.1, 10.0);
  world.setTeamBall(3, {2.0, 0.0, 0.0}, 0.1, 10.0);
  EXPECT_EQ(world.teamBalls().size(), 2u);

  // A teammate reporting again replaces its own entry rather than adding one
  world.setTeamBall(2, {1.5, 0.0, 0.0}, 0.1, 10.5);
  EXPECT_EQ(world.teamBalls().size(), 2u);
}

TEST(WorldModel, ASilentTeammateExpiresOnItsOwn) {
  WorldModel world(makeConfig());
  world.setTeamBall(2, {1.0, 0.0, 0.0}, 0.1, 10.0);
  world.setTeamBall(3, {2.0, 0.0, 0.0}, 0.1, 12.0);

  // Robot 2 went quiet while robot 3 kept talking, so only robot 2's ball ages out
  world.prune(13.5);
  ASSERT_EQ(world.teamBalls().size(), 1u);
  EXPECT_DOUBLE_EQ(world.teamBalls().front().position.x(), 2.0);
}

TEST(WorldModel, TeamBallsAreWeightedByCovariance) {
  WorldModel world(makeConfig());
  world.setTeamBall(2, {1.0, 0.0, 0.0}, 0.5, 10.0);
  ASSERT_EQ(world.teamBalls().size(), 1u);
  EXPECT_DOUBLE_EQ(world.teamBalls().front().weight, 0.5);
}

TEST(WorldModel, TeamBallsCountAsBallInformation) {
  WorldModel world(makeConfig());
  world.setTeamBall(2, {1.0, 0.0, 0.0}, 0.1, 10.0);
  EXPECT_TRUE(world.hasAnyBall());
}

// ---------------------------------------------------------------------------
// Housekeeping
// ---------------------------------------------------------------------------

TEST(WorldModel, PruningIsIdempotent) {
  WorldModel world(makeConfig());
  world.setRawBalls({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});
  world.prune(10.1);
  const size_t after_first = world.rawBalls().size();
  world.prune(10.1);
  EXPECT_EQ(world.rawBalls().size(), after_first);
}

TEST(WorldModel, ClearForgetsEverything) {
  WorldModel world(makeConfig());
  world.setFilteredBall({1.0, 0.0, 0.0}, 0.1, 10.0);
  world.setRawBalls({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});
  world.setRobots({TimedTarget{{1.0, 0.0, 0.0}, 1.0, 10.0}});
  world.setTeamBall(2, {1.0, 0.0, 0.0}, 0.1, 10.0);

  world.clear();
  EXPECT_FALSE(world.hasAnyBall());
  EXPECT_TRUE(world.robots().empty());
}
