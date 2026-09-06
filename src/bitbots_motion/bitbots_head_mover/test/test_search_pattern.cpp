#include <gtest/gtest.h>

#include <algorithm>
#include <bitbots_head_mover/search_pattern.hpp>
#include <cmath>

using bitbots_head_mover::generatePattern;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::interpolatedSteps;
using bitbots_head_mover::lineAngle;

// ---------------------------------------------------------------------------
// lineAngle
// ---------------------------------------------------------------------------

TEST(LineAngle, FirstLineIsMinAngle) { EXPECT_DOUBLE_EQ(lineAngle(0, 5, -5.0, 35.0), -5.0); }

TEST(LineAngle, LastLineIsMaxAngle) { EXPECT_DOUBLE_EQ(lineAngle(4, 5, -5.0, 35.0), 35.0); }

TEST(LineAngle, LinesAreEvenlySpaced) {
  // Four gaps over a 40 degree span means 10 degrees per line
  EXPECT_DOUBLE_EQ(lineAngle(1, 5, -5.0, 35.0), 5.0);
  EXPECT_DOUBLE_EQ(lineAngle(2, 5, -5.0, 35.0), 15.0);
  EXPECT_DOUBLE_EQ(lineAngle(3, 5, -5.0, 35.0), 25.0);
}

TEST(LineAngle, SingleLineCollapsesOntoMinAngle) {
  // Guards against the division by zero that a single scan line would produce
  EXPECT_DOUBLE_EQ(lineAngle(0, 1, -5.0, 35.0), -5.0);
}

TEST(LineAngle, UsesAbsoluteSpanSoOrderOfBoundsDoesNotFlipStepDirection) {
  // The step size is derived from the absolute span, so passing the bounds the
  // other way around still steps upwards from the first argument
  EXPECT_DOUBLE_EQ(lineAngle(1, 3, 35.0, -5.0), 55.0);
}

// ---------------------------------------------------------------------------
// interpolatedSteps
// ---------------------------------------------------------------------------

TEST(InterpolatedSteps, ZeroStepsYieldsNothing) { EXPECT_TRUE(interpolatedSteps(0, 10.0, -30.0, 30.0).empty()); }

TEST(InterpolatedSteps, ReturnsOneMoreThanRequestedSteps) {
  // The implementation adds one step so the upper bound is included
  EXPECT_EQ(interpolatedSteps(3, 10.0, -30.0, 30.0).size(), 4u);
}

TEST(InterpolatedSteps, EndsAtMaxYawAndKeepsPitchConstant) {
  auto steps = interpolatedSteps(3, 10.0, -30.0, 30.0);
  ASSERT_FALSE(steps.empty());
  EXPECT_DOUBLE_EQ(steps.back().yaw, 30.0);
  for (const auto& step : steps) {
    EXPECT_DOUBLE_EQ(step.pitch, 10.0);
  }
}

TEST(InterpolatedSteps, IsEvenlySpacedAndExcludesTheLowerBound) {
  auto steps = interpolatedSteps(3, 0.0, 0.0, 40.0);
  ASSERT_EQ(steps.size(), 4u);
  EXPECT_DOUBLE_EQ(steps[0].yaw, 10.0);
  EXPECT_DOUBLE_EQ(steps[1].yaw, 20.0);
  EXPECT_DOUBLE_EQ(steps[2].yaw, 30.0);
  EXPECT_DOUBLE_EQ(steps[3].yaw, 40.0);
}

TEST(InterpolatedSteps, IsAscendingRegardlessOfBoundOrder) {
  // The span is taken as an absolute value, so the steps always ascend from the
  // first bound. generatePattern relies on this and reverses the result itself.
  auto steps = interpolatedSteps(3, 0.0, 40.0, 0.0);
  ASSERT_EQ(steps.size(), 4u);
  EXPECT_TRUE(std::is_sorted(steps.begin(), steps.end(),
                             [](const HeadPosition& a, const HeadPosition& b) { return a.yaw < b.yaw; }));
}

// ---------------------------------------------------------------------------
// generatePattern
// ---------------------------------------------------------------------------

TEST(GeneratePattern, SingleScanLineStillProducesKeyframes) {
  // A single line clamps the iteration count to its lower bound of two
  auto pattern = generatePattern(1, -30.0, 30.0, -5.0, 35.0);
  EXPECT_EQ(pattern.size(), 2u);
}

TEST(GeneratePattern, SingleScanLineProducesFiniteAngles) {
  // scan_lines is allowed to be one by the parameter validation, and a NaN
  // keyframe would propagate all the way into the published motor goals
  auto pattern = generatePattern(1, -30.0, 30.0, -5.0, 35.0);
  for (const auto& keyframe : pattern) {
    EXPECT_TRUE(std::isfinite(keyframe.yaw));
    EXPECT_TRUE(std::isfinite(keyframe.pitch));
    EXPECT_DOUBLE_EQ(keyframe.pitch, -5.0);
  }
}

TEST(GeneratePattern, KeyframeCountGrowsWithScanLines) {
  // Without interpolation the pattern has exactly max(4 * lines - 4, 2) keyframes
  EXPECT_EQ(generatePattern(2, -30.0, 30.0, -5.0, 35.0).size(), 4u);
  EXPECT_EQ(generatePattern(3, -30.0, 30.0, -5.0, 35.0).size(), 8u);
  EXPECT_EQ(generatePattern(4, -30.0, 30.0, -5.0, 35.0).size(), 12u);
}

TEST(GeneratePattern, StartsAtTheBottomScanLine) {
  auto pattern = generatePattern(3, -30.0, 30.0, -5.0, 35.0);
  ASSERT_FALSE(pattern.empty());
  // The scan starts on the last line, which sits at the "down" angle
  EXPECT_DOUBLE_EQ(pattern.front().pitch, 35.0);
}

TEST(GeneratePattern, StaysWithinTheConfiguredBounds) {
  auto pattern = generatePattern(4, -30.0, 30.0, -5.0, 35.0);
  ASSERT_FALSE(pattern.empty());
  for (const auto& keyframe : pattern) {
    EXPECT_GE(keyframe.yaw, -30.0);
    EXPECT_LE(keyframe.yaw, 30.0);
    EXPECT_GE(keyframe.pitch, -5.0);
    EXPECT_LE(keyframe.pitch, 35.0);
  }
}

TEST(GeneratePattern, VisitsEveryScanLine) {
  const int line_count = 4;
  auto pattern = generatePattern(line_count, -30.0, 30.0, -5.0, 35.0);
  for (int line = 0; line < line_count; line++) {
    const double expected = lineAngle(line, line_count, -5.0, 35.0);
    EXPECT_TRUE(std::any_of(pattern.begin(), pattern.end(), [&](const HeadPosition& keyframe) {
      return std::abs(keyframe.pitch - expected) < 1e-9;
    })) << "scan line "
        << line << " at pitch " << expected << " was never visited";
  }
}

TEST(GeneratePattern, AlternatesHorizontalDirectionBetweenLines) {
  auto pattern = generatePattern(3, -30.0, 30.0, -5.0, 35.0);
  ASSERT_GE(pattern.size(), 4u);
  // Consecutive keyframes on the same line sit on opposite sides
  EXPECT_NE(pattern[0].yaw, pattern[1].yaw);
  EXPECT_DOUBLE_EQ(pattern[0].pitch, pattern[1].pitch);
}

TEST(GeneratePattern, ReducesTheLowestScanLineTowardsTheCenter) {
  const double reduction = 0.2;
  auto pattern = generatePattern(3, -30.0, 30.0, -5.0, 35.0, reduction);
  bool saw_lowest_line = false;
  for (const auto& keyframe : pattern) {
    if (std::abs(keyframe.pitch - 35.0) < 1e-6) {
      saw_lowest_line = true;
      // Looking down and far to the side collides with the body, so those
      // keyframes get pulled towards the center
      EXPECT_LE(std::abs(keyframe.yaw), 30.0 * reduction + 1e-9);
    } else {
      EXPECT_DOUBLE_EQ(std::abs(keyframe.yaw), 30.0);
    }
  }
  EXPECT_TRUE(saw_lowest_line);
}

TEST(GeneratePattern, DefaultReductionLeavesTheLowestScanLineUntouched) {
  auto pattern = generatePattern(3, -30.0, 30.0, -5.0, 35.0);
  for (const auto& keyframe : pattern) {
    EXPECT_DOUBLE_EQ(std::abs(keyframe.yaw), 30.0);
  }
}

TEST(GeneratePattern, InterpolationAddsIntermediateKeyframes) {
  auto without = generatePattern(3, -30.0, 30.0, -5.0, 35.0, 1.0, 0);
  auto with = generatePattern(3, -30.0, 30.0, -5.0, 35.0, 1.0, 3);
  EXPECT_GT(with.size(), without.size());
}

TEST(GeneratePattern, DegenerateBoundsProduceAConstantPattern) {
  // The look forward mode configures a zero width and zero height pattern
  auto pattern = generatePattern(2, 0.0, 0.0, 0.0, 0.0);
  ASSERT_FALSE(pattern.empty());
  for (const auto& keyframe : pattern) {
    EXPECT_DOUBLE_EQ(keyframe.yaw, 0.0);
    EXPECT_DOUBLE_EQ(keyframe.pitch, 0.0);
  }
}
