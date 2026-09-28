#include <gtest/gtest.h>

#include <bitbots_head_mover/look_at.hpp>
#include <cmath>

using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::kCameraPitchOffset;
using bitbots_head_mover::motorGoalsFromPoint;

namespace {
geometry_msgs::msg::Point makePoint(double x, double y, double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
}  // namespace

TEST(MotorGoalsFromPoint, PointStraightAheadKeepsTheYawAtRest) {
  // Passing a zero camera offset isolates the pure geometry
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 0.0, 0.0), makePoint(1.0, 0.0, 0.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.yaw, 0.0, 1e-9);
  EXPECT_NEAR(goal.pitch, 0.0, 1e-9);
}

TEST(MotorGoalsFromPoint, PointToTheLeftYieldsAPositiveYaw) {
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 1.0, 0.0), makePoint(1.0, 1.0, 0.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.yaw, M_PI / 4.0, 1e-9);
}

TEST(MotorGoalsFromPoint, PointToTheRightYieldsANegativeYaw) {
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, -1.0, 0.0), makePoint(1.0, -1.0, 0.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.yaw, -M_PI / 4.0, 1e-9);
}

TEST(MotorGoalsFromPoint, PointBelowYieldsAPositivePitch) {
  // Pitch is positive when looking down, so a point below the joint tilts down
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 0.0, -1.0), makePoint(1.0, 0.0, -1.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.pitch, M_PI / 4.0, 1e-9);
}

TEST(MotorGoalsFromPoint, PointAboveYieldsANegativePitch) {
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 0.0, 1.0), makePoint(1.0, 0.0, 1.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.pitch, -M_PI / 4.0, 1e-9);
}

TEST(MotorGoalsFromPoint, ResultIsRelativeToTheCurrentHeadPosition) {
  const HeadPosition current{0.3, 0.2};
  // A point straight ahead in the joint frames means "keep looking where we look"
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 0.0, 0.0), makePoint(1.0, 0.0, 0.0), current, 0.0);
  EXPECT_NEAR(goal.yaw, current.yaw, 1e-9);
  EXPECT_NEAR(goal.pitch, current.pitch, 1e-9);
}

TEST(MotorGoalsFromPoint, CameraOffsetShiftsThePitchGoalOnly) {
  const auto point = makePoint(1.0, 1.0, -1.0);
  HeadPosition without = motorGoalsFromPoint(point, point, {0.0, 0.0}, 0.0);
  HeadPosition with = motorGoalsFromPoint(point, point, {0.0, 0.0}, kCameraPitchOffset);
  EXPECT_NEAR(with.yaw, without.yaw, 1e-9);
  EXPECT_NEAR(with.pitch, without.pitch - kCameraPitchOffset, 1e-9);
}

TEST(MotorGoalsFromPoint, DefaultOffsetIsTheCameraMountingAngle) {
  const auto point = makePoint(1.0, 0.0, 0.0);
  HeadPosition goal = motorGoalsFromPoint(point, point, {0.0, 0.0});
  EXPECT_NEAR(goal.pitch, -kCameraPitchOffset, 1e-9);
}

TEST(MotorGoalsFromPoint, SolvesEachJointInItsOwnFrame) {
  // The same point expressed in the two joint frames generally differs, and each
  // joint has to be solved against its own version of it
  HeadPosition goal = motorGoalsFromPoint(makePoint(1.0, 1.0, 0.0), makePoint(1.0, 0.0, -1.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(goal.yaw, M_PI / 4.0, 1e-9);
  EXPECT_NEAR(goal.pitch, M_PI / 4.0, 1e-9);
}

TEST(MotorGoalsFromPoint, DistanceDoesNotChangeTheAngles) {
  HeadPosition near = motorGoalsFromPoint(makePoint(1.0, 1.0, -1.0), makePoint(1.0, 1.0, -1.0), {0.0, 0.0}, 0.0);
  HeadPosition far = motorGoalsFromPoint(makePoint(5.0, 5.0, -5.0), makePoint(5.0, 5.0, -5.0), {0.0, 0.0}, 0.0);
  EXPECT_NEAR(near.yaw, far.yaw, 1e-9);
  EXPECT_NEAR(near.pitch, far.pitch, 1e-9);
}
