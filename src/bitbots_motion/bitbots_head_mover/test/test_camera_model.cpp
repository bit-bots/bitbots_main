#include <gtest/gtest.h>

#include <bitbots_head_mover/camera_model.hpp>

using bitbots_head_mover::CameraModel;
using bitbots_head_mover::VisibilityWeighting;

namespace {

/// A pinhole camera with a centered principal point.
///
/// The focal length equals half the width, which puts the horizontal field of
/// view at exactly 90 degrees and makes the pixel coordinates below easy to
/// verify: a point at 45 degrees lands exactly on the image border.
sensor_msgs::msg::CameraInfo makeCameraInfo(uint32_t width = 640, uint32_t height = 480) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  const double fx = width / 2.0;
  const double fy = width / 2.0;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

CameraModel makeModel() {
  CameraModel model;
  model.update(makeCameraInfo());
  return model;
}

}  // namespace

// ---------------------------------------------------------------------------
// Intrinsics
// ---------------------------------------------------------------------------

TEST(CameraModel, IsInvalidBeforeReceivingIntrinsics) {
  CameraModel model;
  EXPECT_FALSE(model.valid());
  EXPECT_FALSE(model.project({0.0, 0.0, 1.0}).has_value());
  EXPECT_DOUBLE_EQ(model.visibility({0.0, 0.0, 1.0}), 0.0);
}

TEST(CameraModel, AcceptsUsableIntrinsics) {
  CameraModel model;
  EXPECT_TRUE(model.update(makeCameraInfo()));
  EXPECT_TRUE(model.valid());
  EXPECT_DOUBLE_EQ(model.width(), 640.0);
  EXPECT_DOUBLE_EQ(model.height(), 480.0);
}

TEST(CameraModel, RejectsAnUncalibratedCamera) {
  CameraModel model;
  sensor_msgs::msg::CameraInfo info = makeCameraInfo();
  // A driver that has not been calibrated publishes a zeroed intrinsic matrix,
  // which would collapse every projection onto the principal point
  info.k = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(model.update(info));
  EXPECT_FALSE(model.valid());
}

TEST(CameraModel, RejectsAZeroSizedImage) {
  CameraModel model;
  sensor_msgs::msg::CameraInfo info = makeCameraInfo();
  info.width = 0;
  EXPECT_FALSE(model.update(info));
  EXPECT_FALSE(model.valid());
}

TEST(CameraModel, StaysValidWhenTheSameIntrinsicsArriveAgain) {
  // image_geometry reports whether the intrinsics changed, which must not be
  // mistaken for the update having failed
  CameraModel model;
  ASSERT_TRUE(model.update(makeCameraInfo()));
  EXPECT_TRUE(model.update(makeCameraInfo()));
  EXPECT_TRUE(model.valid());
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

TEST(CameraModel, ProjectsTheOpticalAxisToTheImageCenter) {
  CameraModel model = makeModel();
  auto pixel = model.project({0.0, 0.0, 1.0});
  ASSERT_TRUE(pixel.has_value());
  EXPECT_NEAR(pixel->x(), 320.0, 1e-6);
  EXPECT_NEAR(pixel->y(), 240.0, 1e-6);
}

TEST(CameraModel, ProjectsAlongTheOpticalFrameAxes) {
  CameraModel model = makeModel();
  // In an optical frame x points right and y points down
  auto right = model.project({0.5, 0.0, 1.0});
  ASSERT_TRUE(right.has_value());
  EXPECT_GT(right->x(), 320.0);
  EXPECT_NEAR(right->y(), 240.0, 1e-6);

  auto down = model.project({0.0, 0.5, 1.0});
  ASSERT_TRUE(down.has_value());
  EXPECT_GT(down->y(), 240.0);
}

TEST(CameraModel, RejectsPointsBehindTheCamera) {
  CameraModel model = makeModel();
  EXPECT_FALSE(model.project({0.0, 0.0, -1.0}).has_value());
  EXPECT_FALSE(model.project({0.0, 0.0, 0.0}).has_value());
}

TEST(CameraModel, RejectsPointsOutsideTheImage) {
  CameraModel model = makeModel();
  // At a focal length of half the width, 45 degrees sits on the image border
  EXPECT_TRUE(model.project({0.99, 0.0, 1.0}).has_value());
  EXPECT_FALSE(model.project({1.01, 0.0, 1.0}).has_value());
}

TEST(CameraModel, DistanceDoesNotChangeTheProjection) {
  CameraModel model = makeModel();
  auto near = model.project({0.2, 0.1, 1.0});
  auto far = model.project({2.0, 1.0, 10.0});
  ASSERT_TRUE(near.has_value());
  ASSERT_TRUE(far.has_value());
  EXPECT_NEAR(near->x(), far->x(), 1e-6);
  EXPECT_NEAR(near->y(), far->y(), 1e-6);
}

// ---------------------------------------------------------------------------
// Visibility scoring
// ---------------------------------------------------------------------------

TEST(CameraModel, InvisiblePointsScoreZero) {
  CameraModel model = makeModel();
  EXPECT_DOUBLE_EQ(model.visibility({0.0, 0.0, -1.0}), 0.0);
  EXPECT_DOUBLE_EQ(model.visibility({5.0, 0.0, 1.0}), 0.0);
}

TEST(CameraModel, CenteredPointsScorePerfectly) {
  CameraModel model = makeModel();
  EXPECT_DOUBLE_EQ(model.visibility({0.0, 0.0, 1.0}), 1.0);
}

TEST(CameraModel, TheWholeCentralRegionScoresPerfectly) {
  CameraModel model = makeModel();
  const VisibilityWeighting weighting{0.5, 0.3};
  // Half of the half width is a quarter of the image width from the center,
  // which is just inside the central 50% of the image
  EXPECT_DOUBLE_EQ(model.visibility({0.49, 0.0, 1.0}, weighting), 1.0);
  // Just outside it the score starts to drop
  EXPECT_LT(model.visibility({0.55, 0.0, 1.0}, weighting), 1.0);
}

TEST(CameraModel, ScoreFallsOffTowardsTheBorder) {
  CameraModel model = makeModel();
  const VisibilityWeighting weighting{0.5, 0.3};
  const double center = model.visibility({0.0, 0.0, 1.0}, weighting);
  const double middle = model.visibility({0.75, 0.0, 1.0}, weighting);
  const double border = model.visibility({0.999, 0.0, 1.0}, weighting);
  EXPECT_GT(center, middle);
  EXPECT_GT(middle, border);
}

TEST(CameraModel, BorderPointsScoreTheConfiguredBorderScore) {
  CameraModel model = makeModel();
  const VisibilityWeighting weighting{0.5, 0.3};
  // A point essentially on the image border keeps the head interested in it,
  // but much less so than a centered one
  EXPECT_NEAR(model.visibility({0.9999, 0.0, 1.0}, weighting), 0.3, 1e-3);
}

TEST(CameraModel, BothAxesCountTowardsBeingCentered) {
  CameraModel model = makeModel();
  const VisibilityWeighting weighting{0.5, 0.3};
  // Centered horizontally but near the top edge is not well framed. The vertical
  // half angle is smaller, so a modest y already reaches the image edge.
  EXPECT_LT(model.visibility({0.0, 0.7, 1.0}, weighting), 1.0);
}

TEST(CameraModel, CenterFractionWidensThePerfectRegion) {
  CameraModel model = makeModel();
  const Eigen::Vector3d point{0.75, 0.0, 1.0};
  EXPECT_LT(model.visibility(point, {0.5, 0.3}), 1.0);
  EXPECT_DOUBLE_EQ(model.visibility(point, {0.9, 0.3}), 1.0);
}

TEST(CameraModel, ScoreStaysWithinTheUnitInterval) {
  CameraModel model = makeModel();
  for (double x = -1.5; x <= 1.5; x += 0.1) {
    for (double y = -1.5; y <= 1.5; y += 0.1) {
      const double score = model.visibility({x, y, 1.0});
      EXPECT_GE(score, 0.0);
      EXPECT_LE(score, 1.0);
    }
  }
}
