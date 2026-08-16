#include <gtest/gtest.h>

#include <bitbots_head_mover/active_vision_scorer.hpp>
#include <bitbots_head_mover/trajectory_sampler.hpp>
#include <cmath>

using bitbots_head_mover::ActiveVisionScorer;
using bitbots_head_mover::buildCandidateTrajectory;
using bitbots_head_mover::CameraModel;
using bitbots_head_mover::DynamicLimits;
using bitbots_head_mover::FieldCoverageConfig;
using bitbots_head_mover::FieldCoverageMap;
using bitbots_head_mover::HeadChainConfig;
using bitbots_head_mover::HeadKinematics;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::HeadTrajectory;
using bitbots_head_mover::ScoringContext;
using bitbots_head_mover::ScoringWeights;
using bitbots_head_mover::TimedTarget;
using bitbots_head_mover::WorldModel;

namespace {

/// A head whose camera looks forward along the robot's x axis when at rest.
///
/// The optical frame convention has z forward, x right and y down, so the fixed
/// joint at the end of the chain rotates the link frame accordingly. Without
/// that rotation every projection would be behind the camera.
constexpr const char* kHeadUrdf = R"(<?xml version="1.0"?>
<robot name="test_head">
  <link name="base_link"/>
  <link name="head_yaw_link"/>
  <link name="head_pitch_link"/>
  <link name="camera_optical_frame_left_uncalibrated"/>

  <joint name="head_yaw_joint" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="head_yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.43" upper="1.43" effort="3" velocity="30.37"/>
  </joint>
  <joint name="head_pitch_joint" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="head_yaw_link"/>
    <child link="head_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.23" upper="1.01" effort="3" velocity="30.37"/>
  </joint>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
    <parent link="head_pitch_link"/>
    <child link="camera_optical_frame_left_uncalibrated"/>
  </joint>
</robot>)";

sensor_msgs::msg::CameraInfo makeCameraInfo() {
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  const double fx = 320.0;
  const double cx = 320.0;
  const double cy = 240.0;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

FieldCoverageConfig makeCoverageConfig() {
  FieldCoverageConfig config;
  config.field_length = 9.0;
  config.field_width = 6.0;
  config.margin = 1.0;
  config.cell_size = 0.5;
  config.half_life = 8.0;
  return config;
}

/// A trajectory that holds a single head position for the whole horizon.
HeadTrajectory holdAt(const HeadPosition& position) {
  return buildCandidateTrajectory(position, {}, position, position, 0.5, 1.0, DynamicLimits{});
}

ScoringContext makeContext() {
  ScoringContext context;
  // The robot stands in the center of the field looking down the long axis
  context.robot_pose = Eigen::Isometry3d::Identity();
  context.evaluation_times = {0.0, 0.25, 0.5, 0.75, 1.0};
  return context;
}

/// Bundles everything the scorer needs so the tests stay readable.
struct Fixture {
  std::unique_ptr<HeadKinematics> kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  CameraModel camera;
  WorldModel world;
  FieldCoverageMap coverage{makeCoverageConfig()};

  Fixture() { camera.update(makeCameraInfo()); }

  ActiveVisionScorer scorer(const Eigen::Isometry3d& robot_pose = Eigen::Isometry3d::Identity()) {
    return ActiveVisionScorer(*kinematics, camera, world, coverage, robot_pose);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Preconditions
// ---------------------------------------------------------------------------

TEST(ActiveVisionScorer, ScoresZeroWithoutIntrinsics) {
  Fixture fixture;
  CameraModel blind;
  ActiveVisionScorer scorer(*fixture.kinematics, blind, fixture.world, fixture.coverage, Eigen::Isometry3d::Identity());
  EXPECT_DOUBLE_EQ(scorer.score(holdAt({0.0, 0.0}), makeContext()).total, 0.0);
}

TEST(ActiveVisionScorer, ScoresZeroForAnInvalidCandidate) {
  Fixture fixture;
  EXPECT_DOUBLE_EQ(fixture.scorer().score(HeadTrajectory(), makeContext()).total, 0.0);
}

TEST(ActiveVisionScorer, ScoresZeroWithoutEvaluationTimes) {
  Fixture fixture;
  ScoringContext context = makeContext();
  context.evaluation_times.clear();
  EXPECT_DOUBLE_EQ(fixture.scorer().score(holdAt({0.0, 0.0}), context).total, 0.0);
}

// ---------------------------------------------------------------------------
// Ball terms
// ---------------------------------------------------------------------------

TEST(ActiveVisionScorer, PrefersLookingAtTheFilteredBall) {
  Fixture fixture;
  // A ball on the ground two meters in front of the robot
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 0.0, 0.0);
  auto scorer = fixture.scorer();

  // Looking down towards the ball beats looking away from it
  const double towards = scorer.score(holdAt({0.0, 0.45}), makeContext()).filtered_ball;
  const double away = scorer.score(holdAt({-1.2, 0.0}), makeContext()).filtered_ball;
  EXPECT_GT(towards, 0.0);
  EXPECT_GT(towards, away);
}

TEST(ActiveVisionScorer, AnUncertainBallContributesLess) {
  Fixture fixture;
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 0.0, 0.0);
  const double certain = fixture.scorer().score(holdAt({0.0, 0.45}), makeContext()).filtered_ball;

  // The very same ball, but the filter is far less sure about it
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 5.0, 0.0);
  const double uncertain = fixture.scorer().score(holdAt({0.0, 0.45}), makeContext()).filtered_ball;

  EXPECT_GT(certain, uncertain);
}

TEST(ActiveVisionScorer, NoBallMeansNoBallScore) {
  Fixture fixture;
  EXPECT_DOUBLE_EQ(fixture.scorer().score(holdAt({0.0, 0.45}), makeContext()).filtered_ball, 0.0);
}

TEST(ActiveVisionScorer, RawBallDetectionsAreScoredSeparately) {
  Fixture fixture;
  fixture.world.setRawBalls({TimedTarget{{2.0, 0.0, 0.0}, 1.0, 0.0}});
  auto breakdown = fixture.scorer().score(holdAt({0.0, 0.45}), makeContext());
  EXPECT_GT(breakdown.raw_balls, 0.0);
  EXPECT_DOUBLE_EQ(breakdown.filtered_ball, 0.0);
}

TEST(ActiveVisionScorer, TeamBallsAreScoredSeparately) {
  Fixture fixture;
  fixture.world.setTeamBall(2, {2.0, 0.0, 0.0}, 0.0, 0.0);
  auto breakdown = fixture.scorer().score(holdAt({0.0, 0.45}), makeContext());
  EXPECT_GT(breakdown.team_ball, 0.0);
  EXPECT_DOUBLE_EQ(breakdown.raw_balls, 0.0);
}

TEST(ActiveVisionScorer, RobotDetectionsAreScoredSeparately) {
  Fixture fixture;
  fixture.world.setRobots({TimedTarget{{2.0, 0.0, 0.3}, 1.0, 0.0}});
  auto breakdown = fixture.scorer().score(holdAt({0.0, 0.4}), makeContext());
  EXPECT_GT(breakdown.robots, 0.0);
}

TEST(ActiveVisionScorer, CenteringABallBeatsCatchingItAtTheEdge) {
  Fixture fixture;
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 0.0, 0.0);
  auto scorer = fixture.scorer();

  double best = 0.0;
  double best_pitch = 0.0;
  for (double pitch = 0.0; pitch < 1.0; pitch += 0.02) {
    const double score = scorer.score(holdAt({0.0, pitch}), makeContext()).filtered_ball;
    if (score > best) {
      best = score;
      best_pitch = pitch;
    }
  }
  // The optimum actually centers the ball rather than merely keeping it visible
  EXPECT_DOUBLE_EQ(best, 1.0);
  EXPECT_GT(best_pitch, 0.0);
}

// ---------------------------------------------------------------------------
// Coverage and out of field
// ---------------------------------------------------------------------------

TEST(ActiveVisionScorer, UnobservedFieldIsWorthLookingAt) {
  Fixture fixture;
  EXPECT_GT(fixture.scorer().score(holdAt({0.0, 0.4}), makeContext()).field_coverage, 0.0);
}

TEST(ActiveVisionScorer, AlreadyObservedFieldIsWorthLess) {
  Fixture fixture;
  const auto candidate = holdAt({0.0, 0.4});
  const double before = fixture.scorer().score(candidate, makeContext()).field_coverage;

  // Record what that very head position sees, then ask again
  fixture.scorer().recordObservation(fixture.coverage, {0.0, 0.4}, makeContext().robot_pose);
  const double after = fixture.scorer().score(candidate, makeContext()).field_coverage;

  EXPECT_GT(before, 0.0);
  EXPECT_LT(after, before);
}

TEST(ActiveVisionScorer, ObservationsDecayBackIntoInterest) {
  Fixture fixture;
  const auto candidate = holdAt({0.0, 0.4});
  const double fresh = fixture.scorer().score(candidate, makeContext()).field_coverage;

  fixture.scorer().recordObservation(fixture.coverage, {0.0, 0.4}, makeContext().robot_pose);
  const double observed = fixture.scorer().score(candidate, makeContext()).field_coverage;

  // After a long while the same part of the field is interesting again
  fixture.coverage.decay(60.0);
  const double stale = fixture.scorer().score(candidate, makeContext()).field_coverage;

  EXPECT_LT(observed, fresh);
  EXPECT_GT(stale, observed);
}

TEST(ActiveVisionScorer, LookingOffTheFieldEarnsNoCoverage) {
  Fixture fixture;

  // Standing near the touch line, so turning one way faces the field and the
  // other way faces the area beyond it
  ScoringContext context = makeContext();
  context.robot_pose.translation() = Eigen::Vector3d(0.0, 2.5, 0.0);
  auto scorer = fixture.scorer(context.robot_pose);

  const double towards_field = scorer.score(holdAt({-1.2, 0.4}), context).field_coverage;
  const double away_from_field = scorer.score(holdAt({1.2, 0.4}), context).field_coverage;

  // Off field cells carry no interest, so aiming at them simply earns nothing.
  // That opportunity cost is what replaces the former explicit penalty.
  EXPECT_GT(towards_field, away_from_field);
}

TEST(ActiveVisionScorer, LookingAtTheSkyEarnsNoCoverage) {
  Fixture fixture;
  auto scorer = fixture.scorer();

  // Pitched fully up nothing projects into the image at all. An explicit off
  // field penalty measured as a share of visible cells would score this as
  // perfectly clean, which made staring at the sky better than glancing past
  // the touch line. Earning no coverage is the behavior that ranks it last.
  const double sky = scorer.score(holdAt({0.0, -1.2}), makeContext()).field_coverage;
  const double field = scorer.score(holdAt({0.0, 0.4}), makeContext()).field_coverage;

  EXPECT_DOUBLE_EQ(sky, 0.0);
  EXPECT_GT(field, sky);
}

TEST(ActiveVisionScorer, DistantFieldCountsForLessThanNearField) {
  Fixture fixture;

  // Two cells of field, one close to the robot and one far away, both unseen.
  // A view of the far one sweeps up more ground area simply because distance
  // packs more of it into the same image, so without a falloff it would win.
  ScoringContext context = makeContext();
  auto scorer = fixture.scorer(context.robot_pose);

  // Looking down sees the near ground, looking towards the horizon sees far ground
  const double near_view = scorer.score(holdAt({0.0, 0.8}), context).field_coverage;
  const double far_view = scorer.score(holdAt({0.0, 0.1}), context).field_coverage;

  // Both see field, but the near view must not be dwarfed by the far one
  EXPECT_GT(near_view, 0.0);
  EXPECT_GT(far_view, 0.0);
}

TEST(ActiveVisionScorer, TheDistanceFalloffDampensFarViewpoints) {
  Fixture fixture;
  ScoringContext context = makeContext();

  // A short falloff discounts distance hard, a long one barely at all
  auto sharp = fixture.scorer(context.robot_pose);
  sharp.setCoverageDistanceHalfWeight(1.0);
  auto flat = fixture.scorer(context.robot_pose);
  flat.setCoverageDistanceHalfWeight(100.0);
  sharp.prepare(context.robot_pose);
  flat.prepare(context.robot_pose);

  // A view aimed towards the horizon, which is where the far cells are
  const auto far_view = holdAt({0.0, 0.1});
  const auto near_view = holdAt({0.0, 0.8});

  const double sharp_ratio = sharp.score(far_view, context).field_coverage /
                             std::max(sharp.score(near_view, context).field_coverage, 1e-12);
  const double flat_ratio = flat.score(far_view, context).field_coverage /
                            std::max(flat.score(near_view, context).field_coverage, 1e-12);

  // Discounting distance more sharply has to move the balance towards the near view
  EXPECT_LT(sharp_ratio, flat_ratio);
}

// ---------------------------------------------------------------------------
// Commitment
// ---------------------------------------------------------------------------

TEST(ActiveVisionScorer, AgreeingWithThePreviousSelectionScoresHigher) {
  Fixture fixture;
  auto scorer = fixture.scorer();

  ScoringContext context = makeContext();
  context.previous_positions.assign(context.evaluation_times.size(), HeadPosition{0.5, 0.2});

  const double same = scorer.score(holdAt({0.5, 0.2}), context).commitment;
  const double different = scorer.score(holdAt({-0.5, -0.2}), context).commitment;
  EXPECT_DOUBLE_EQ(same, 1.0);
  EXPECT_LT(different, same);
}

TEST(ActiveVisionScorer, NoPreviousSelectionMeansNoCommitment) {
  Fixture fixture;
  // Without a previous trajectory the term must not favour any candidate
  EXPECT_DOUBLE_EQ(fixture.scorer().score(holdAt({0.5, 0.2}), makeContext()).commitment, 0.0);
}

TEST(ActiveVisionScorer, CommitmentStaysWithinTheUnitInterval) {
  Fixture fixture;
  auto scorer = fixture.scorer();
  ScoringContext context = makeContext();
  context.previous_positions.assign(context.evaluation_times.size(), HeadPosition{1.2, 1.0});

  for (double yaw = -1.2; yaw <= 1.2; yaw += 0.2) {
    const double commitment = scorer.score(holdAt({yaw, 0.0}), context).commitment;
    EXPECT_GE(commitment, 0.0);
    EXPECT_LE(commitment, 1.0);
  }
}

// ---------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------

TEST(ActiveVisionScorer, EveryTermStaysWithinTheUnitInterval) {
  Fixture fixture;
  fixture.world.setFilteredBall({2.0, 0.5, 0.0}, 0.2, 0.0);
  fixture.world.setRawBalls({TimedTarget{{2.0, 0.5, 0.0}, 1.0, 0.0}});
  fixture.world.setTeamBall(2, {-2.0, 0.0, 0.0}, 0.2, 0.0);
  fixture.world.setRobots({TimedTarget{{1.0, 1.0, 0.3}, 1.0, 0.0}});
  auto scorer = fixture.scorer();

  ScoringContext context = makeContext();
  context.previous_positions.assign(context.evaluation_times.size(), HeadPosition{0.0, 0.3});

  for (double yaw = -1.2; yaw <= 1.2; yaw += 0.3) {
    for (double pitch = -1.0; pitch <= 1.0; pitch += 0.25) {
      const auto breakdown = scorer.score(holdAt({yaw, pitch}), context);
      for (double term : {breakdown.filtered_ball, breakdown.raw_balls, breakdown.team_ball, breakdown.field_coverage,
                          breakdown.robots, breakdown.commitment}) {
        EXPECT_GE(term, 0.0);
        EXPECT_LE(term, 1.0);
      }
    }
  }
}

TEST(ActiveVisionScorer, TheTotalIsTheWeightedSumOfTheTerms) {
  Fixture fixture;
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 0.1, 0.0);
  auto scorer = fixture.scorer();
  const ScoringWeights& weights = scorer.weights();

  const auto breakdown = scorer.score(holdAt({0.0, 0.4}), makeContext());
  const double expected = weights.filtered_ball * breakdown.filtered_ball + weights.raw_balls * breakdown.raw_balls +
                          weights.team_ball * breakdown.team_ball +
                          weights.field_coverage * breakdown.field_coverage + weights.robots * breakdown.robots +
                          weights.commitment * breakdown.commitment;
  EXPECT_NEAR(breakdown.total, expected, 1e-12);
}

TEST(ActiveVisionScorer, ZeroWeightsDisableATerm) {
  Fixture fixture;
  fixture.world.setFilteredBall({2.0, 0.0, 0.0}, 0.0, 0.0);
  auto scorer = fixture.scorer();

  ScoringWeights weights;
  weights.filtered_ball = 0.0;
  weights.raw_balls = 0.0;
  weights.team_ball = 0.0;
  weights.field_coverage = 0.0;
  weights.robots = 0.0;
  weights.commitment = 0.0;
  scorer.setWeights(weights);

  EXPECT_DOUBLE_EQ(scorer.score(holdAt({0.0, 0.4}), makeContext()).total, 0.0);
}

TEST(ActiveVisionScorer, TheRobotPoseMovesWhatIsVisible) {
  Fixture fixture;
  fixture.world.setFilteredBall({4.0, 0.0, 0.0}, 0.0, 0.0);
  auto scorer = fixture.scorer();

  // Standing a meter short of the ball and facing it
  ScoringContext facing = makeContext();
  facing.robot_pose.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);

  // Standing in the very same spot, but turned around
  ScoringContext turned_away = makeContext();
  turned_away.robot_pose.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  turned_away.robot_pose.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // The very same head position sees very different things depending on how the
  // robot stands, which is exactly why the map frame is used throughout
  const double towards = scorer.score(holdAt({0.0, 0.5}), facing).filtered_ball;
  const double away = scorer.score(holdAt({0.0, 0.5}), turned_away).filtered_ball;
  EXPECT_GT(towards, 0.0);
  EXPECT_DOUBLE_EQ(away, 0.0);
}
