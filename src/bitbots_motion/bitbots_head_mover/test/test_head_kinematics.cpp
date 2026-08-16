#include <gtest/gtest.h>

#include <bitbots_head_mover/head_kinematics.hpp>
#include <cmath>

using bitbots_head_mover::HeadChainConfig;
using bitbots_head_mover::HeadKinematics;
using bitbots_head_mover::HeadPosition;

namespace {

/// A minimal stand-in for the head chain of the real robot.
///
/// The yaw joint sits one meter above the root and turns around z, the pitch
/// joint sits in the same place and turns around y, and the camera is mounted
/// ten centimeters in front of it. That makes every pose below verifiable by
/// hand, which a chain taken from the real robot description would not be.
constexpr const char* kHeadUrdf = R"(<?xml version="1.0"?>
<robot name="test_head">
  <link name="base_link"/>
  <link name="torso_link"/>
  <link name="head_yaw_link"/>
  <link name="head_pitch_link"/>
  <link name="camera_optical_frame_left_uncalibrated"/>

  <joint name="torso_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="torso_link"/>
  </joint>
  <joint name="head_yaw_joint" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="torso_link"/>
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
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <parent link="head_pitch_link"/>
    <child link="camera_optical_frame_left_uncalibrated"/>
  </joint>
</robot>)";

/// A chain whose head joints are joined by a third movable joint.
constexpr const char* kExtraJointUrdf = R"(<?xml version="1.0"?>
<robot name="test_head">
  <link name="base_link"/>
  <link name="head_yaw_link"/>
  <link name="extra_link"/>
  <link name="head_pitch_link"/>
  <link name="camera_optical_frame_left_uncalibrated"/>

  <joint name="head_yaw_joint" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="head_yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.43" upper="1.43" effort="3" velocity="30.37"/>
  </joint>
  <joint name="extra_joint" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="head_yaw_link"/>
    <child link="extra_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="3" velocity="30.37"/>
  </joint>
  <joint name="head_pitch_joint" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="extra_link"/>
    <child link="head_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.23" upper="1.01" effort="3" velocity="30.37"/>
  </joint>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <parent link="head_pitch_link"/>
    <child link="camera_optical_frame_left_uncalibrated"/>
  </joint>
</robot>)";

}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(HeadKinematics, BuildsFromAValidDescription) { EXPECT_NE(HeadKinematics::fromUrdf(kHeadUrdf), nullptr); }

TEST(HeadKinematics, RejectsMalformedDescriptions) {
  EXPECT_EQ(HeadKinematics::fromUrdf("this is not a urdf"), nullptr);
  EXPECT_EQ(HeadKinematics::fromUrdf(""), nullptr);
}

TEST(HeadKinematics, RejectsAMissingLink) {
  HeadChainConfig config;
  config.tip_link = "a_link_that_does_not_exist";
  EXPECT_EQ(HeadKinematics::fromUrdf(kHeadUrdf, config), nullptr);
}

TEST(HeadKinematics, RejectsAMissingJoint) {
  HeadChainConfig config;
  config.yaw_joint = "a_joint_that_does_not_exist";
  EXPECT_EQ(HeadKinematics::fromUrdf(kHeadUrdf, config), nullptr);
}

TEST(HeadKinematics, RejectsChainsWithAdditionalMovableJoints) {
  // A joint we do not control would move the camera without us knowing, which
  // would silently invalidate every sampled camera pose
  EXPECT_EQ(HeadKinematics::fromUrdf(kExtraJointUrdf), nullptr);
}

TEST(HeadKinematics, ReadsTheJointLimitsFromTheDescription) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);
  EXPECT_DOUBLE_EQ(kinematics->urdfLimits().yaw.lower, -1.43);
  EXPECT_DOUBLE_EQ(kinematics->urdfLimits().yaw.upper, 1.43);
  EXPECT_DOUBLE_EQ(kinematics->urdfLimits().pitch.lower, -1.23);
  EXPECT_DOUBLE_EQ(kinematics->urdfLimits().pitch.upper, 1.01);
}

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

TEST(HeadKinematics, RestPoseSitsInFrontOfTheJoints) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  const auto pose = kinematics->cameraPose({0.0, 0.0});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->translation().x(), 0.1, 1e-9);
  EXPECT_NEAR(pose->translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().z(), 1.0, 1e-9);
  EXPECT_TRUE(pose->linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(HeadKinematics, YawRotatesTheCameraAroundTheVerticalAxis) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  // Turning the head a quarter turn to the left swings the camera to the side
  const auto pose = kinematics->cameraPose({M_PI / 2.0, 0.0});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().y(), 0.1, 1e-9);
  EXPECT_NEAR(pose->translation().z(), 1.0, 1e-9);
}

TEST(HeadKinematics, PitchTiltsTheCameraDown) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  // A positive pitch turns around y, which tips the forward axis downwards
  const auto pose = kinematics->cameraPose({0.0, M_PI / 2.0});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().z(), 0.9, 1e-9);
}

TEST(HeadKinematics, YawAndPitchComposeInTheRightOrder) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  // Pitch acts in the frame the yaw joint already rotated, so tipping fully down
  // puts the camera below the joints no matter how the head is turned
  const auto pose = kinematics->cameraPose({M_PI / 2.0, M_PI / 2.0});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(pose->translation().z(), 0.9, 1e-9);
}

TEST(HeadKinematics, PoseIsAValidRigidTransform) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  const auto pose = kinematics->cameraPose({0.4, -0.3});
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->linear().determinant(), 1.0, 1e-9);
  EXPECT_TRUE((pose->linear() * pose->linear().transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(HeadKinematics, EvaluatesPositionsTheRobotIsNotIn) {
  // The whole point of the component: two different configurations have to give
  // two different camera poses without the robot ever moving
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  const auto left = kinematics->cameraPose({0.5, 0.0});
  ASSERT_TRUE(left.has_value());
  const auto right = kinematics->cameraPose({-0.5, 0.0});
  ASSERT_TRUE(right.has_value());
  EXPECT_GT(left->translation().y(), right->translation().y());
}

// ---------------------------------------------------------------------------
// Camera calibration
// ---------------------------------------------------------------------------

TEST(HeadKinematics, HasNoCalibrationByDefault) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);
  EXPECT_FALSE(kinematics->hasCameraCalibration());
}

TEST(HeadKinematics, CalibrationIsAppliedInTheCameraFrame) {
  auto kinematics = HeadKinematics::fromUrdf(kHeadUrdf);
  ASSERT_NE(kinematics, nullptr);

  // The calibration is expressed relative to the uncalibrated optical frame, so
  // it has to be composed onto the right hand side of the chain
  Eigen::Isometry3d calibration = Eigen::Isometry3d::Identity();
  calibration.translation() = Eigen::Vector3d(0.0, 0.0, 0.05);
  kinematics->setCameraCalibration(calibration);
  EXPECT_TRUE(kinematics->hasCameraCalibration());

  // At rest the camera frame is aligned with the root, so the offset shows up on z
  const auto rest = kinematics->cameraPose({0.0, 0.0});
  ASSERT_TRUE(rest.has_value());
  EXPECT_NEAR(rest->translation().z(), 1.05, 1e-9);

  // Turned a quarter turn to the left the very same offset still points along
  // the camera's own z, which now points along the root's z as well
  const auto turned = kinematics->cameraPose({M_PI / 2.0, 0.0});
  ASSERT_TRUE(turned.has_value());
  EXPECT_NEAR(turned->translation().z(), 1.05, 1e-9);

  // Tipped fully down, the camera's z points backwards along the root's x
  const auto tipped = kinematics->cameraPose({0.0, M_PI / 2.0});
  ASSERT_TRUE(tipped.has_value());
  EXPECT_NEAR(tipped->translation().x(), 0.05, 1e-9);
  EXPECT_NEAR(tipped->translation().z(), 0.9, 1e-9);
}
