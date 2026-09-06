#include <gtest/gtest.h>

#include <bitbots_head_mover/head_trajectory.hpp>
#include <cmath>

using bitbots_head_mover::adjustSpeeds;
using bitbots_head_mover::buildSearchPatternTrajectory;
using bitbots_head_mover::calculateLowerSpeed;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::HeadTrajectory;
using bitbots_head_mover::HeadVelocity;
using bitbots_head_mover::SearchPatternTrajectory;

namespace {
constexpr double kDegToRad = M_PI / 180.0;

/// A simple two line pattern in degrees, as generatePattern would produce it.
const std::vector<HeadPosition> kPattern = {{-30.0, 35.0}, {30.0, 35.0}, {30.0, -5.0}, {-30.0, -5.0}};

/// The first pattern keyframe as a head position, i.e. converted to radians.
/// Starting there means there is no transition segment to play.
const HeadPosition kPatternStart{kPattern[0].yaw * kDegToRad, kPattern[0].pitch * kDegToRad};
}  // namespace

// ---------------------------------------------------------------------------
// HeadTrajectory
// ---------------------------------------------------------------------------

TEST(HeadTrajectory, IsInvalidBeforeFinalize) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.addPoint(1.0, {1.0, 0.5});
  EXPECT_FALSE(trajectory.valid());
}

TEST(HeadTrajectory, IsInvalidWithASingleWaypoint) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.finalize();
  EXPECT_FALSE(trajectory.valid());
}

TEST(HeadTrajectory, InterpolatesBetweenWaypoints) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.addPoint(1.0, {1.0, 0.5});
  trajectory.finalize();

  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.position(0.0).yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.position(0.0).pitch, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.position(1.0).yaw, 1.0, 1e-9);
  EXPECT_NEAR(trajectory.position(1.0).pitch, 0.5, 1e-9);
  // The quintic spline is monotonic between two resting waypoints
  EXPECT_GT(trajectory.position(0.5).yaw, 0.0);
  EXPECT_LT(trajectory.position(0.5).yaw, 1.0);
}

TEST(HeadTrajectory, RespectsTheWaypointVelocities) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {0.0, 0.0}, {2.0, -1.0});
  trajectory.addPoint(1.0, {1.0, 0.5});
  trajectory.finalize();

  ASSERT_TRUE(trajectory.valid());
  EXPECT_NEAR(trajectory.velocity(0.0).yaw, 2.0, 1e-9);
  EXPECT_NEAR(trajectory.velocity(0.0).pitch, -1.0, 1e-9);
  // The terminal velocity defaults to rest
  EXPECT_NEAR(trajectory.velocity(1.0).yaw, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.velocity(1.0).pitch, 0.0, 1e-9);
}

TEST(HeadTrajectory, VelocityIsSigned) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {1.0, 0.0});
  trajectory.addPoint(1.0, {-1.0, 0.0});
  trajectory.finalize();

  ASSERT_TRUE(trajectory.valid());
  // Moving towards a smaller yaw means a negative velocity, the motor goals
  // take the absolute value themselves
  EXPECT_LT(trajectory.velocity(0.5).yaw, 0.0);
}

TEST(HeadTrajectory, ReportsDurationAndSize) {
  HeadTrajectory trajectory;
  trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.addPoint(0.5, {1.0, 0.0});
  trajectory.addPoint(2.5, {0.0, 0.0});
  trajectory.finalize();

  EXPECT_EQ(trajectory.size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 2.5);
}

TEST(HeadTrajectory, InvalidTrajectoryEvaluatesToZero) {
  HeadTrajectory trajectory;
  EXPECT_DOUBLE_EQ(trajectory.position(1.0).yaw, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.velocity(1.0).yaw, 0.0);
}

// ---------------------------------------------------------------------------
// buildSearchPatternTrajectory
// ---------------------------------------------------------------------------

TEST(BuildSearchPatternTrajectory, EmptyPatternIsInvalid) {
  EXPECT_FALSE(buildSearchPatternTrajectory({}, 2.0, {0.0, 0.0}, 6.0).valid());
}

TEST(BuildSearchPatternTrajectory, NonPositiveCycleTimeIsInvalid) {
  EXPECT_FALSE(buildSearchPatternTrajectory(kPattern, 0.0, {0.0, 0.0}, 6.0).valid());
  EXPECT_FALSE(buildSearchPatternTrajectory(kPattern, -1.0, {0.0, 0.0}, 6.0).valid());
}

TEST(BuildSearchPatternTrajectory, NonPositiveTransitionSpeedIsInvalid) {
  // Guards against dividing the transition distance by zero
  EXPECT_FALSE(buildSearchPatternTrajectory(kPattern, 2.0, {0.0, 0.0}, 0.0).valid());
}

TEST(BuildSearchPatternTrajectory, ConvertsThePatternToRadians) {
  auto result = buildSearchPatternTrajectory(kPattern, 2.0, kPatternStart, 6.0);
  ASSERT_TRUE(result.valid());
  // Starting at the first keyframe means there is no transition to play
  EXPECT_NEAR(result.transition_duration, 0.0, 1e-12);
  EXPECT_NEAR(result.trajectory.position(0.0).yaw, -30.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(result.trajectory.position(0.0).pitch, 35.0 * kDegToRad, 1e-9);
}

TEST(BuildSearchPatternTrajectory, CycleDurationMatchesTheRequestedCycleTime) {
  auto result = buildSearchPatternTrajectory(kPattern, 2.0, kPatternStart, 6.0);
  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.cycle_duration, 2.0);
  // The pattern is closed, so the trajectory ends where it started
  EXPECT_DOUBLE_EQ(result.trajectory.duration(), result.transition_duration + 2.0);
}

TEST(BuildSearchPatternTrajectory, ClosesTheLoop) {
  auto result = buildSearchPatternTrajectory(kPattern, 2.0, kPatternStart, 6.0);
  ASSERT_TRUE(result.valid());
  HeadPosition start = result.trajectory.position(result.transition_duration);
  HeadPosition end = result.trajectory.position(result.trajectory.duration());
  EXPECT_NEAR(start.yaw, end.yaw, 1e-9);
  EXPECT_NEAR(start.pitch, end.pitch, 1e-9);
}

TEST(BuildSearchPatternTrajectory, PrependsATransitionFromTheCurrentPosition) {
  const HeadPosition start{0.0, 0.0};
  auto result = buildSearchPatternTrajectory(kPattern, 2.0, start, 6.0);
  ASSERT_TRUE(result.valid());

  // The transition takes as long as the distance divided by the transition speed
  const double dyaw = -30.0 * kDegToRad - start.yaw;
  const double dpitch = 35.0 * kDegToRad - start.pitch;
  const double expected = std::sqrt(dyaw * dyaw + dpitch * dpitch) / 6.0;
  EXPECT_NEAR(result.transition_duration, expected, 1e-9);

  // ... and it actually starts at the current head position
  EXPECT_NEAR(result.trajectory.position(0.0).yaw, start.yaw, 1e-9);
  EXPECT_NEAR(result.trajectory.position(0.0).pitch, start.pitch, 1e-9);
}

TEST(BuildSearchPatternTrajectory, TransitionSpeedScalesTheTransitionDuration) {
  auto slow = buildSearchPatternTrajectory(kPattern, 2.0, {0.0, 0.0}, 3.0);
  auto fast = buildSearchPatternTrajectory(kPattern, 2.0, {0.0, 0.0}, 6.0);
  ASSERT_TRUE(slow.valid());
  ASSERT_TRUE(fast.valid());
  EXPECT_NEAR(slow.transition_duration, 2.0 * fast.transition_duration, 1e-9);
}

TEST(BuildSearchPatternTrajectory, DistributesTimeProportionallyToDistance) {
  // A pattern whose second leg is three times as long as the first one
  const std::vector<HeadPosition> pattern = {{0.0, 0.0}, {10.0, 0.0}, {40.0, 0.0}};
  auto result = buildSearchPatternTrajectory(pattern, 8.0, pattern[0], 6.0);
  ASSERT_TRUE(result.valid());

  // Legs are 10, 30 and 40 degrees long, so the second keyframe is reached
  // after 10/80 of the cycle and the third after 40/80
  EXPECT_NEAR(result.trajectory.position(8.0 * 10.0 / 80.0).yaw, 10.0 * kDegToRad, 1e-9);
  EXPECT_NEAR(result.trajectory.position(8.0 * 40.0 / 80.0).yaw, 40.0 * kDegToRad, 1e-9);
}

TEST(BuildSearchPatternTrajectory, DegeneratePatternDistributesTimeEvenly) {
  // The look forward pattern collapses onto a single point, so there is no
  // distance to distribute time by and the fallback has to avoid a zero division
  const std::vector<HeadPosition> pattern = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  auto result = buildSearchPatternTrajectory(pattern, 3.0, {0.0, 0.0}, 6.0);
  ASSERT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.cycle_duration, 3.0);
  EXPECT_TRUE(std::isfinite(result.trajectory.position(1.5).yaw));
  EXPECT_NEAR(result.trajectory.position(1.5).yaw, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// SearchPatternTrajectory::phase
// ---------------------------------------------------------------------------

TEST(SearchPatternPhase, PassesTheTransitionThroughOnce) {
  SearchPatternTrajectory trajectory;
  trajectory.transition_duration = 1.0;
  trajectory.cycle_duration = 4.0;
  trajectory.trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.trajectory.addPoint(5.0, {1.0, 0.0});
  trajectory.trajectory.finalize();
  ASSERT_TRUE(trajectory.valid());

  EXPECT_DOUBLE_EQ(trajectory.phase(0.0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.phase(0.5), 0.5);
  EXPECT_DOUBLE_EQ(trajectory.phase(1.0), 1.0);
}

TEST(SearchPatternPhase, LoopsOnlyTheCyclicPart) {
  SearchPatternTrajectory trajectory;
  trajectory.transition_duration = 1.0;
  trajectory.cycle_duration = 4.0;
  trajectory.trajectory.addPoint(0.0, {0.0, 0.0});
  trajectory.trajectory.addPoint(5.0, {1.0, 0.0});
  trajectory.trajectory.finalize();
  ASSERT_TRUE(trajectory.valid());

  // Just after the transition we are at its end
  EXPECT_NEAR(trajectory.phase(1.5), 1.5, 1e-12);
  // One full cycle later we are back where the cycle started
  EXPECT_NEAR(trajectory.phase(5.5), 1.5, 1e-12);
  // And the transition is never replayed
  EXPECT_GE(trajectory.phase(100.0), 1.0);
  EXPECT_LE(trajectory.phase(100.0), 5.0);
}

TEST(SearchPatternPhase, InvalidTrajectoryStaysAtZero) {
  SearchPatternTrajectory trajectory;
  EXPECT_DOUBLE_EQ(trajectory.phase(3.0), 0.0);
}

// ---------------------------------------------------------------------------
// Speed adjustment
// ---------------------------------------------------------------------------

TEST(CalculateLowerSpeed, ScalesWithTheDistanceRatio) {
  // The faster joint takes 2 seconds for its 4 units, so the other joint has to
  // cover its 2 units in the same time
  EXPECT_DOUBLE_EQ(calculateLowerSpeed(4.0, 2.0, 2.0), 1.0);
}

TEST(CalculateLowerSpeed, ReturnsZeroWhenTheFasterJointDoesNotMove) {
  EXPECT_DOUBLE_EQ(calculateLowerSpeed(0.0, 2.0, 2.0), 0.0);
}

TEST(AdjustSpeeds, SlowsDownThePitchAxisWhenYawTravelsFurther) {
  HeadVelocity speeds = adjustSpeeds({1.0, 0.25}, {0.0, 0.0}, {2.0, 2.0});
  // Yaw keeps its full speed, pitch is scaled by the distance ratio
  EXPECT_DOUBLE_EQ(speeds.yaw, 2.0);
  EXPECT_DOUBLE_EQ(speeds.pitch, 0.5);
}

TEST(AdjustSpeeds, SlowsDownTheYawAxisWhenPitchTravelsFurther) {
  HeadVelocity speeds = adjustSpeeds({0.25, 1.0}, {0.0, 0.0}, {2.0, 2.0});
  EXPECT_DOUBLE_EQ(speeds.pitch, 2.0);
  EXPECT_DOUBLE_EQ(speeds.yaw, 0.5);
}

TEST(AdjustSpeeds, NeverExceedsTheGivenMaximum) {
  // The shorter traveling joint would be allowed to go faster than its maximum
  HeadVelocity speeds = adjustSpeeds({10.0, 1.0}, {0.0, 0.0}, {1.0, 1.0});
  EXPECT_LE(speeds.yaw, 1.0);
  EXPECT_LE(speeds.pitch, 1.0);
}

TEST(AdjustSpeeds, BothJointsArriveTogether) {
  const HeadPosition goal{1.0, 0.25};
  const HeadPosition current{0.0, 0.0};
  HeadVelocity speeds = adjustSpeeds(goal, current, {2.0, 2.0});
  const double yaw_time = std::abs(goal.yaw - current.yaw) / speeds.yaw;
  const double pitch_time = std::abs(goal.pitch - current.pitch) / speeds.pitch;
  EXPECT_NEAR(yaw_time, pitch_time, 1e-9);
}

TEST(AdjustSpeeds, ZeroDistanceLeavesTheOtherJointAtZero) {
  // Nothing to move means the derived speed collapses to zero
  HeadVelocity speeds = adjustSpeeds({0.0, 0.0}, {0.0, 0.0}, {2.0, 2.0});
  EXPECT_DOUBLE_EQ(speeds.yaw, 0.0);
}
