#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "bitbots_vision/candidate.hpp"
#include "bitbots_vision/debug_image.hpp"

using bitbots_vision::Candidate;
using bitbots_vision::DebugImage;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static cv::Mat make_image(int h = 100, int w = 100)
{
  cv::Mat img(h, w, CV_8UC3, cv::Scalar(128, 128, 128));
  return img;
}

static Candidate make_candidate(int x1, int y1, int x2, int y2, float rating = 0.9f)
{
  return Candidate::from_x1y1x2y2(x1, y1, x2, y2, rating);
}

// ---------------------------------------------------------------------------
// Inactive DebugImage – all draw methods are no-ops
// ---------------------------------------------------------------------------

TEST(DebugImage, Inactive_GetImageIsEmpty)
{
  DebugImage di(false);
  EXPECT_TRUE(di.get_image().empty());
}

TEST(DebugImage, Inactive_DrawBallCandidates_NoOp)
{
  DebugImage di(false);
  // Should not crash even without set_image()
  EXPECT_NO_THROW(di.draw_ball_candidates({make_candidate(10, 10, 30, 30)}, DebugImage::kBall));
}

TEST(DebugImage, Inactive_DrawBoxCandidates_NoOp)
{
  DebugImage di(false);
  EXPECT_NO_THROW(
    di.draw_box_candidates({make_candidate(10, 10, 30, 30)}, DebugImage::kGoalposts));
}

TEST(DebugImage, Inactive_DrawMask_NoOp)
{
  DebugImage di(false);
  cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(255));
  EXPECT_NO_THROW(di.draw_mask(mask, DebugImage::kLines));
}

// ---------------------------------------------------------------------------
// set_image – copies the input
// ---------------------------------------------------------------------------

TEST(DebugImage, SetImage_PreservesSize)
{
  DebugImage di(true);
  cv::Mat src = make_image(200, 300);
  di.set_image(src);
  EXPECT_EQ(di.get_image().rows, 200);
  EXPECT_EQ(di.get_image().cols, 300);
}

TEST(DebugImage, SetImage_IsCopy)
{
  DebugImage di(true);
  cv::Mat src = make_image(50, 50);
  di.set_image(src);
  // Mutating src afterwards should not affect debug image
  src.setTo(cv::Scalar(0, 0, 0));
  EXPECT_NE(cv::sum(di.get_image())[0], 0.0);
}

// ---------------------------------------------------------------------------
// draw_ball_candidates
// ---------------------------------------------------------------------------

TEST(DebugImage, DrawBallCandidates_Empty_NoChange)
{
  DebugImage di(true);
  cv::Mat src = make_image();
  di.set_image(src);

  cv::Mat before = di.get_image().clone();
  di.draw_ball_candidates({}, DebugImage::kBall, 2);

  // Pixel-wise unchanged
  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(DebugImage, DrawBallCandidates_ModifiesImage)
{
  DebugImage di(true);
  di.set_image(make_image(200, 200));

  cv::Mat before = di.get_image().clone();
  di.draw_ball_candidates({make_candidate(50, 50, 100, 100)}, DebugImage::kBall, 2);

  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(DebugImage, DrawBallCandidates_MultipleNoThrow)
{
  DebugImage di(true);
  di.set_image(make_image(300, 400));

  std::vector<Candidate> cands = {
    make_candidate(10, 10, 40, 40),
    make_candidate(100, 100, 150, 150),
    make_candidate(200, 50, 250, 100),
  };
  EXPECT_NO_THROW(di.draw_ball_candidates(cands, DebugImage::kBall, 1));
}

// ---------------------------------------------------------------------------
// draw_box_candidates
// ---------------------------------------------------------------------------

TEST(DebugImage, DrawBoxCandidates_ModifiesImage)
{
  DebugImage di(true);
  di.set_image(make_image(200, 200));

  cv::Mat before = di.get_image().clone();
  di.draw_box_candidates({make_candidate(20, 20, 80, 80)}, DebugImage::kGoalposts, 2);

  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(DebugImage, DrawBoxCandidates_Empty_NoChange)
{
  DebugImage di(true);
  cv::Mat src = make_image();
  di.set_image(src);
  cv::Mat before = di.get_image().clone();

  di.draw_box_candidates({}, DebugImage::kRobotUnknown, 1);

  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

// ---------------------------------------------------------------------------
// draw_mask
// ---------------------------------------------------------------------------

TEST(DebugImage, DrawMask_AllActive_BlendChangesPixels)
{
  DebugImage di(true);
  // Use a uniform dark image so the green overlay is clearly visible
  di.set_image(cv::Mat(100, 100, CV_8UC3, cv::Scalar(0, 0, 0)));

  cv::Mat before = di.get_image().clone();

  cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(255));  // fully active
  di.draw_mask(mask, DebugImage::kLines);  // blue overlay

  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(DebugImage, DrawMask_AllZero_NoChange)
{
  DebugImage di(true);
  di.set_image(make_image());
  cv::Mat before = di.get_image().clone();

  cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(0));  // nothing active
  di.draw_mask(mask, DebugImage::kLines);

  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(DebugImage, DrawMask_PartialMask_OnlyActiveRegionChanged)
{
  DebugImage di(true);
  di.set_image(cv::Mat(100, 100, CV_8UC3, cv::Scalar(50, 50, 50)));

  cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(0));
  // Only top-left quadrant is active
  mask(cv::Rect(0, 0, 50, 50)).setTo(255);

  cv::Mat before = di.get_image().clone();
  di.draw_mask(mask, DebugImage::kBall);

  // Count changed pixels inside the active region
  cv::Mat diff;
  cv::absdiff(di.get_image(), before, diff);
  cv::Mat active_diff = diff(cv::Rect(0, 0, 50, 50));
  cv::Mat inactive_diff = diff(cv::Rect(50, 50, 50, 50));

  EXPECT_GT(cv::countNonZero(active_diff.reshape(1)), 0);
  EXPECT_EQ(cv::countNonZero(inactive_diff.reshape(1)), 0);
}

// ---------------------------------------------------------------------------
// Color constants sanity
// ---------------------------------------------------------------------------

TEST(DebugImage, ColorConstants_AreDistinct)
{
  // Just verify the constants are accessible and differ from each other
  EXPECT_NE(DebugImage::kBall, DebugImage::kGoalposts);
  EXPECT_NE(DebugImage::kRobotTeamMates, DebugImage::kRobotOpponents);
  EXPECT_NE(DebugImage::kLines, DebugImage::kBall);
}
