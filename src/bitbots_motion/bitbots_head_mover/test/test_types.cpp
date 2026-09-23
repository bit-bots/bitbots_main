#include <gtest/gtest.h>

#include <bitbots_head_mover/types.hpp>

using bitbots_head_mover::HeadLimits;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::JointLimit;

namespace {
/// The head limits the robot is configured with by default.
constexpr HeadLimits kDefaultLimits{{-1.23, 1.23}, {-1.23, 1.01}};
}  // namespace

// ---------------------------------------------------------------------------
// JointLimit
// ---------------------------------------------------------------------------

TEST(JointLimit, ClampLeavesInteriorValuesUntouched) {
  constexpr JointLimit limit{-1.0, 1.0};
  EXPECT_DOUBLE_EQ(limit.clamp(0.5), 0.5);
  EXPECT_DOUBLE_EQ(limit.clamp(-0.5), -0.5);
}

TEST(JointLimit, ClampPullsExteriorValuesToTheBound) {
  constexpr JointLimit limit{-1.0, 1.0};
  EXPECT_DOUBLE_EQ(limit.clamp(2.0), 1.0);
  EXPECT_DOUBLE_EQ(limit.clamp(-2.0), -1.0);
}

TEST(JointLimit, ContainsIsExclusiveOnTheBounds) {
  constexpr JointLimit limit{-1.0, 1.0};
  EXPECT_TRUE(limit.contains(0.0));
  // A goal sitting exactly on a limit has always been rejected as a collision
  EXPECT_FALSE(limit.contains(1.0));
  EXPECT_FALSE(limit.contains(-1.0));
  EXPECT_FALSE(limit.contains(1.5));
}

// ---------------------------------------------------------------------------
// HeadLimits
// ---------------------------------------------------------------------------

TEST(HeadLimits, ClampAppliesPerJoint) {
  HeadPosition clamped = kDefaultLimits.clamp({5.0, -5.0});
  EXPECT_DOUBLE_EQ(clamped.yaw, 1.23);
  EXPECT_DOUBLE_EQ(clamped.pitch, -1.23);
}

TEST(HeadLimits, ClampUsesTheAsymmetricPitchBounds) {
  // Pitch is not symmetric, looking up is limited more than looking down
  EXPECT_DOUBLE_EQ(kDefaultLimits.clamp({0.0, 5.0}).pitch, 1.01);
  EXPECT_DOUBLE_EQ(kDefaultLimits.clamp({0.0, -5.0}).pitch, -1.23);
}

TEST(HeadLimits, ContainsRequiresBothJointsInRange) {
  EXPECT_TRUE(kDefaultLimits.contains({0.0, 0.0}));
  EXPECT_FALSE(kDefaultLimits.contains({2.0, 0.0}));
  EXPECT_FALSE(kDefaultLimits.contains({0.0, 2.0}));
  EXPECT_FALSE(kDefaultLimits.contains({2.0, 2.0}));
}

TEST(HeadLimits, ClampedPositionsAreNotNecessarilyContained) {
  // Clamping puts a position exactly onto the bound, which contains() rejects.
  // The node therefore has to clip and collision check independently.
  HeadPosition clamped = kDefaultLimits.clamp({5.0, 0.0});
  EXPECT_FALSE(kDefaultLimits.contains(clamped));
}
