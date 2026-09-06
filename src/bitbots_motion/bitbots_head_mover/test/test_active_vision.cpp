#include <gtest/gtest.h>

#include <bitbots_head_mover/active_vision.hpp>
#include <bitbots_head_mover/active_vision_debug.hpp>
#include <cmath>
#include <opencv2/imgproc.hpp>

using bitbots_head_mover::ActiveVision;
using bitbots_head_mover::ActiveVisionInput;
using bitbots_head_mover::ActiveVisionReadiness;
using bitbots_head_mover::ActiveVisionResult;
using bitbots_head_mover::FieldCoverageConfig;
using bitbots_head_mover::HeadChainConfig;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::SamplerConfig;
using bitbots_head_mover::ScoringWeights;
using bitbots_head_mover::TimedTarget;

namespace {

/// The same minimal head chain the scorer tests use, with the camera looking
/// forward along the robot's x axis when the head is at rest.
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
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
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

SamplerConfig makeSamplerConfig() {
  SamplerConfig config;
  config.sample_count = 64;
  config.last_target_weight = 0.4;
  config.current_position_weight = 0.3;
  config.uniform_weight = 0.3;
  config.last_target_std = 0.3;
  config.current_position_std = 0.5;
  return config;
}

/// A planner with every input satisfied.
ActiveVision makeReadyPlanner() {
  ActiveVision planner;
  planner.setRobotDescription(kHeadUrdf, HeadChainConfig{});
  planner.setCameraInfo(makeCameraInfo());
  planner.setFieldCoverageConfig(makeCoverageConfig());
  planner.setSamplerConfig(makeSamplerConfig());
  return planner;
}

ActiveVisionInput makeInput(double now = 0.0) {
  ActiveVisionInput input;
  input.head_position = {0.0, 0.0};
  input.robot_pose = Eigen::Isometry3d::Identity();
  input.now = now;
  return input;
}

/// Whether a position lies within the inclusive joint bounds.
bool withinLimits(const HeadPosition& position, const bitbots_head_mover::HeadLimits& limits) {
  return position.yaw >= limits.yaw.lower && position.yaw <= limits.yaw.upper && position.pitch >= limits.pitch.lower &&
         position.pitch <= limits.pitch.upper;
}

}  // namespace

// ---------------------------------------------------------------------------
// Readiness
// ---------------------------------------------------------------------------

TEST(ActiveVision, IsNotReadyWithoutAnyInputs) {
  ActiveVision planner;
  EXPECT_FALSE(planner.ready());
  EXPECT_EQ(planner.readiness(), ActiveVisionReadiness::MissingRobotDescription);
}

TEST(ActiveVision, ReportsTheMissingCameraInfo) {
  ActiveVision planner;
  ASSERT_TRUE(planner.setRobotDescription(kHeadUrdf, HeadChainConfig{}));
  EXPECT_EQ(planner.readiness(), ActiveVisionReadiness::MissingCameraInfo);
}

TEST(ActiveVision, ReportsTheMissingFieldDimensions) {
  ActiveVision planner;
  ASSERT_TRUE(planner.setRobotDescription(kHeadUrdf, HeadChainConfig{}));
  ASSERT_TRUE(planner.setCameraInfo(makeCameraInfo()));
  EXPECT_EQ(planner.readiness(), ActiveVisionReadiness::MissingFieldDimensions);
}

TEST(ActiveVision, IsReadyOnceEveryInputArrived) {
  ActiveVision planner = makeReadyPlanner();
  EXPECT_TRUE(planner.ready());
  EXPECT_EQ(planner.readiness(), ActiveVisionReadiness::Ready);
}

TEST(ActiveVision, RejectsAnUnusableRobotDescription) {
  ActiveVision planner;
  EXPECT_FALSE(planner.setRobotDescription("not a urdf", HeadChainConfig{}));
  EXPECT_FALSE(planner.ready());
}

TEST(ActiveVision, DoesNotPlanWhileUnready) {
  ActiveVision planner;
  EXPECT_FALSE(planner.plan(makeInput()).valid);
}

// ---------------------------------------------------------------------------
// Calibration handling
// ---------------------------------------------------------------------------

TEST(ActiveVision, CalibrationSurvivesALateRobotDescription) {
  ActiveVision planner;
  Eigen::Isometry3d calibration = Eigen::Isometry3d::Identity();
  calibration.translation() = Eigen::Vector3d(0.0, 0.0, 0.05);

  // The calibration transform can be available before the description is
  planner.setCameraCalibration(calibration);
  ASSERT_TRUE(planner.setRobotDescription(kHeadUrdf, HeadChainConfig{}));

  EXPECT_TRUE(planner.kinematics().hasCameraCalibration());
}

// ---------------------------------------------------------------------------
// Planning
// ---------------------------------------------------------------------------

TEST(ActiveVision, PlansACommand) {
  ActiveVision planner = makeReadyPlanner();
  const ActiveVisionResult result = planner.plan(makeInput());

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.candidates.empty());
  EXPECT_EQ(result.scores.size(), result.candidates.size());
  EXPECT_LT(result.selected, result.candidates.size());
  EXPECT_TRUE(withinLimits(result.position, planner.headLimits()));
  EXPECT_TRUE(withinLimits(result.target, planner.headLimits()));
}

TEST(ActiveVision, SelectsTheHighestScoringCandidate) {
  ActiveVision planner = makeReadyPlanner();
  const ActiveVisionResult result = planner.plan(makeInput());

  ASSERT_TRUE(result.valid);
  for (const auto& score : result.scores) {
    EXPECT_LE(score.total, result.scores[result.selected].total);
  }
}

TEST(ActiveVision, TheSelectedTargetIsTheSelectedCandidate) {
  ActiveVision planner = makeReadyPlanner();
  const ActiveVisionResult result = planner.plan(makeInput());

  ASSERT_TRUE(result.valid);
  const HeadPosition& selected = result.candidates[result.selected].target;
  EXPECT_NEAR(result.target.yaw, selected.yaw, 1e-9);
  EXPECT_NEAR(result.target.pitch, selected.pitch, 1e-9);
}

TEST(ActiveVision, StepsTowardsTheTargetWithoutOvershooting) {
  ActiveVision planner = makeReadyPlanner();
  // A ball far to the side gives the planner a clear reason to turn the head
  planner.world().setFilteredBall({3.0, 3.0, 0.0}, 0.0, 0.0);

  const ActiveVisionResult result = planner.plan(makeInput());
  ASSERT_TRUE(result.valid);
  ASSERT_GT(std::abs(result.target.yaw), 0.0);

  // The proportional controller commands a setpoint that has moved from the
  // current position towards the target, but not past it
  EXPECT_GT(result.position.yaw * result.target.yaw, 0.0);
  EXPECT_LE(std::abs(result.position.yaw), std::abs(result.target.yaw));
  EXPECT_GT(std::abs(result.position.yaw), 0.0);
}

TEST(ActiveVision, TurnsTowardsTheBall) {
  ActiveVision planner = makeReadyPlanner();
  ScoringWeights weights;
  // Isolate the ball term so the coverage sweep cannot outvote it
  weights.field_coverage = 0.0;
  weights.smoothness = 0.0;
  planner.setScoringWeights(weights);

  // A ball clearly off to the robot's left
  planner.world().setFilteredBall({2.0, 2.0, 0.0}, 0.0, 0.0);

  const ActiveVisionResult result = planner.plan(makeInput());
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.target.yaw, 0.0);
}

TEST(ActiveVision, TurnsTheOtherWayForABallOnTheOtherSide) {
  ActiveVision planner = makeReadyPlanner();
  ScoringWeights weights;
  weights.field_coverage = 0.0;
  weights.smoothness = 0.0;
  planner.setScoringWeights(weights);

  planner.world().setFilteredBall({2.0, -2.0, 0.0}, 0.0, 0.0);

  const ActiveVisionResult result = planner.plan(makeInput());
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.target.yaw, 0.0);
}

TEST(ActiveVision, AgesOutDetectionsWhilePlanning) {
  ActiveVision planner = makeReadyPlanner();
  planner.world().setRawBalls({TimedTarget{{2.0, 0.0, 0.0}, 1.0, 0.0}});

  planner.plan(makeInput(0.1));
  EXPECT_FALSE(planner.world().rawBalls().empty());

  // Well past the raw detection timeout
  planner.plan(makeInput(5.0));
  EXPECT_TRUE(planner.world().rawBalls().empty());
}

TEST(ActiveVision, RecordsWhatTheHeadIsLookingAt) {
  ActiveVision planner = makeReadyPlanner();
  const double before = planner.coverage().totalInterest();

  ActiveVisionInput input = makeInput();
  input.head_position = {0.0, 0.4};
  planner.plan(input);

  // Having looked somewhere has to reduce the outstanding interest, otherwise
  // the coverage term could never steer the head anywhere new
  EXPECT_LT(planner.coverage().totalInterest(), before);
}

TEST(ActiveVision, CoverageRecoversOverTime) {
  ActiveVision planner = makeReadyPlanner();
  ActiveVisionInput input = makeInput(0.0);
  input.head_position = {0.0, 0.4};
  planner.plan(input);
  const double after_looking = planner.coverage().totalInterest();

  // A long time later the same area is worth looking at again
  ActiveVisionInput later = makeInput(120.0);
  later.head_position = {0.0, -1.2};
  planner.plan(later);
  EXPECT_GT(planner.coverage().totalInterest(), after_looking);
}

TEST(ActiveVision, SmoothnessKeepsConsecutiveTargetsTogether) {
  ActiveVision steady = makeReadyPlanner();
  ScoringWeights strong;
  strong.smoothness = 50.0;
  steady.setScoringWeights(strong);

  ActiveVision flighty = makeReadyPlanner();
  ScoringWeights none;
  none.smoothness = 0.0;
  flighty.setScoringWeights(none);

  double steady_travel = 0.0;
  double flighty_travel = 0.0;
  HeadPosition steady_previous{0.0, 0.0};
  HeadPosition flighty_previous{0.0, 0.0};

  for (int step = 1; step <= 20; step++) {
    ActiveVisionInput input = makeInput(step * 0.05);

    input.head_position = steady_previous;
    const auto a = steady.plan(input);
    ASSERT_TRUE(a.valid);
    steady_travel += std::hypot(a.target.yaw - steady_previous.yaw, a.target.pitch - steady_previous.pitch);
    steady_previous = a.target;

    input.head_position = flighty_previous;
    const auto b = flighty.plan(input);
    ASSERT_TRUE(b.valid);
    flighty_travel += std::hypot(b.target.yaw - flighty_previous.yaw, b.target.pitch - flighty_previous.pitch);
    flighty_previous = b.target;
  }

  // This is the whole point of the smoothness cost: without it the head chases
  // whichever target happens to win this cycle and jitters
  EXPECT_LT(steady_travel, flighty_travel);
}

TEST(ActiveVision, ResetForgetsTheHistory) {
  ActiveVision planner = makeReadyPlanner();
  ActiveVisionInput input = makeInput();
  input.head_position = {0.0, 0.4};
  planner.plan(input);
  planner.world().setFilteredBall({2.0, 0.0, 0.0}, 0.0, 0.0);

  // What a planner that never looked anywhere would report
  ActiveVision untouched = makeReadyPlanner();
  const double fresh_interest = untouched.coverage().totalInterest();

  planner.reset();
  EXPECT_FALSE(planner.world().hasAnyBall());
  EXPECT_NEAR(planner.coverage().totalInterest(), fresh_interest, 1e-9);
}

// ---------------------------------------------------------------------------
// Debug rendering
// ---------------------------------------------------------------------------

TEST(ActiveVisionDebug, CoverageGridMatchesTheMap) {
  ActiveVision planner = makeReadyPlanner();
  builtin_interfaces::msg::Time stamp;
  const auto grid = bitbots_head_mover::coverageGrid(planner.coverage(), "map", stamp);

  EXPECT_EQ(grid.header.frame_id, "map");
  EXPECT_EQ(grid.info.width, planner.coverage().cellsX());
  EXPECT_EQ(grid.info.height, planner.coverage().cellsY());
  EXPECT_EQ(grid.data.size(), planner.coverage().size());
  EXPECT_DOUBLE_EQ(grid.info.resolution, makeCoverageConfig().cell_size);
}

TEST(ActiveVisionDebug, CoverageGridValuesAreValidOccupancy) {
  ActiveVision planner = makeReadyPlanner();
  builtin_interfaces::msg::Time stamp;
  const auto grid = bitbots_head_mover::coverageGrid(planner.coverage(), "map", stamp);
  for (int8_t value : grid.data) {
    EXPECT_GE(value, 0);
    EXPECT_LE(value, 100);
  }
}

TEST(ActiveVisionDebug, CandidateMarkersClearThePreviousCycle) {
  ActiveVision planner = makeReadyPlanner();
  const auto result = planner.plan(makeInput());
  builtin_interfaces::msg::Time stamp;
  const auto markers =
      bitbots_head_mover::candidateMarkers(result, planner, Eigen::Isometry3d::Identity(), "map", stamp);

  ASSERT_FALSE(markers.markers.empty());
  // Without a delete all, candidates from a cycle with more samples would linger
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
}

TEST(ActiveVisionDebug, JointSpaceImageHasTheRequestedSize) {
  ActiveVision planner = makeReadyPlanner();
  const auto result = planner.plan(makeInput());
  const cv::Mat image = bitbots_head_mover::jointSpaceDebugImage(result, planner.headLimits(), 400);

  EXPECT_EQ(image.rows, 400);
  EXPECT_EQ(image.cols, 400);
  EXPECT_EQ(image.type(), CV_8UC3);
}

TEST(ActiveVisionDebug, JointSpaceImageDrawsTheCandidates) {
  ActiveVision planner = makeReadyPlanner();
  const auto result = planner.plan(makeInput());
  const cv::Mat with_candidates = bitbots_head_mover::jointSpaceDebugImage(result, planner.headLimits(), 400);
  const cv::Mat without_candidates =
      bitbots_head_mover::jointSpaceDebugImage(ActiveVisionResult(), planner.headLimits(), 400);

  // The plot has to actually show something, otherwise it is useless for tuning
  cv::Mat difference;
  cv::absdiff(with_candidates, without_candidates, difference);
  cv::Mat grey;
  cv::cvtColor(difference, grey, cv::COLOR_BGR2GRAY);
  EXPECT_GT(cv::countNonZero(grey), 0);
}

TEST(ActiveVisionDebug, JointSpaceImageHandlesAnEmptyResult) {
  ActiveVision planner = makeReadyPlanner();
  const cv::Mat image = bitbots_head_mover::jointSpaceDebugImage(ActiveVisionResult(), planner.headLimits(), 128);
  EXPECT_EQ(image.rows, 128);
}
