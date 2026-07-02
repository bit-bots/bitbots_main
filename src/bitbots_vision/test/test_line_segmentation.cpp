#include <gtest/gtest.h>

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/line_segmentation.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace bitbots_vision::processing;
using bitbots_vision::Candidate;

namespace {

LineSegmentationParams make_default_params() {
  LineSegmentationParams p;
  // OpenCV HSV ranges: H in [0,179], S/V in [0,255]
  p.white.low = cv::Scalar(0, 0, 200);
  p.white.high = cv::Scalar(179, 60, 255);
  p.green.low = cv::Scalar(35, 40, 40);
  p.green.high = cv::Scalar(85, 255, 255);
  p.morph_kernel_size = 1;  // disable opening by default so tests are precise
  p.min_contour_area = 0;   // disable contour filtering by default
  return p;
}

// BGR values that map into the default white/green HSV ranges above.
const cv::Scalar kWhiteBgr(255, 255, 255);
const cv::Scalar kGreenBgr(0, 180, 0);
const cv::Scalar kBlackBgr(0, 0, 0);

}  // namespace

// ===========================================================================
// compute_white_green_masks
// ===========================================================================

TEST(ComputeWhiteGreenMasks, ThresholdsMatchExpectedRegions) {
  cv::Mat bgr(10, 10, CV_8UC3, kBlackBgr);
  bgr(cv::Rect(0, 0, 5, 10)) = kWhiteBgr;
  bgr(cv::Rect(5, 0, 5, 10)) = kGreenBgr;

  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, make_default_params(), white_mask, green_mask);

  EXPECT_GT(cv::countNonZero(white_mask(cv::Rect(0, 0, 5, 10))), 0);
  EXPECT_EQ(cv::countNonZero(white_mask(cv::Rect(5, 0, 5, 10))), 0);
  EXPECT_GT(cv::countNonZero(green_mask(cv::Rect(5, 0, 5, 10))), 0);
  EXPECT_EQ(cv::countNonZero(green_mask(cv::Rect(0, 0, 5, 10))), 0);
}

// ===========================================================================
// compute_field_hull_mask
// ===========================================================================

TEST(ComputeFieldHullMask, SolidBlock_HullMatchesBlock) {
  const int size = 50;
  cv::Mat green_mask = cv::Mat::zeros(size, size, CV_8UC1);
  green_mask(cv::Rect(10, 10, 20, 20)) = 255;

  auto hull_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);

  // Convex hull of an axis-aligned rectangle is itself, so the hull mask
  // should closely match the original block.
  EXPECT_GT(hull_mask.at<uint8_t>(20, 20), 0);
  EXPECT_EQ(hull_mask.at<uint8_t>(0, 0), 0);
  EXPECT_EQ(hull_mask.at<uint8_t>(45, 45), 0);
}

TEST(ComputeFieldHullMask, FillsHoleInMiddle_LikeALine) {
  const int size = 50;
  cv::Mat green_mask = cv::Mat::zeros(size, size, CV_8UC1);
  green_mask(cv::Rect(5, 5, 40, 40)) = 255;
  // Cut a "line" shaped hole out of the middle of the green blob.
  green_mask(cv::Rect(24, 5, 2, 40)) = 0;

  auto hull_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);

  // RETR_EXTERNAL ignores internal holes, so the convex hull of the outer
  // boundary must be solid -- including the hole -- unlike the raw green
  // mask itself.
  EXPECT_EQ(green_mask.at<uint8_t>(25, 25), 0);
  EXPECT_GT(hull_mask.at<uint8_t>(25, 25), 0);
}

TEST(ComputeFieldHullMask, IgnoresSmallDisconnectedSpecks) {
  const int size = 100;
  cv::Mat green_mask = cv::Mat::zeros(size, size, CV_8UC1);
  // Main field blob (larger).
  green_mask(cv::Rect(0, 0, 10, 10)) = 255;
  // Small, disconnected green speck far away (e.g. background color drift).
  green_mask(cv::Rect(90, 90, 2, 2)) = 255;

  auto hull_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);

  // The hull must be based on the larger blob only -- the far-away point
  // between the two blobs must not be swept into the hull.
  EXPECT_EQ(hull_mask.at<uint8_t>(50, 50), 0);
  EXPECT_GT(hull_mask.at<uint8_t>(5, 5), 0);
}

TEST(ComputeFieldHullMask, EmptyGreenMask_ReturnsAllZero) {
  cv::Mat green_mask = cv::Mat::zeros(30, 30, CV_8UC1);
  auto hull_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  EXPECT_EQ(cv::countNonZero(hull_mask), 0);
}

TEST(ComputeFieldHullMask, BorderMarginErodesHullInward) {
  const int size = 50;
  cv::Mat green_mask = cv::Mat::zeros(size, size, CV_8UC1);
  green_mask(cv::Rect(10, 10, 20, 20)) = 255;  // hull spans roughly x,y in [10,30)

  auto hull_no_margin = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  auto hull_with_margin = compute_field_hull_mask(green_mask, /*border_margin_px=*/5);

  // A point just inside the original hull's edge is covered without erosion...
  EXPECT_GT(hull_no_margin.at<uint8_t>(20, 11), 0);
  // ...but excluded once the border is eroded inward by more than its
  // distance from the edge.
  EXPECT_EQ(hull_with_margin.at<uint8_t>(20, 11), 0);
  // The interior, far from any edge, is unaffected either way.
  EXPECT_GT(hull_with_margin.at<uint8_t>(20, 20), 0);
}

// ===========================================================================
// compute_line_mask
// ===========================================================================

TEST(ComputeLineMask, LineInsideFieldHull_Survives) {
  const int size = 40;
  cv::Mat bgr(size, size, CV_8UC3, kGreenBgr);
  // Vertical white stripe in the middle -- a "hole" in the green field.
  bgr(cv::Rect(size / 2 - 1, 0, 2, size)) = kWhiteBgr;

  auto params = make_default_params();
  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, params, white_mask, green_mask);
  auto field_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  auto line_mask = compute_line_mask(white_mask, field_mask, green_mask, params.morph_kernel_size, params.min_contour_area);

  EXPECT_GT(cv::countNonZero(line_mask(cv::Rect(size / 2 - 1, 5, 2, size - 10))), 0);
}

TEST(ComputeLineMask, BackgroundWhite_OutsideFieldHull_Suppressed) {
  // The green field is a solid block in the lower half of the image; a
  // white block sits entirely in the upper half, outside the field's convex
  // hull -- it must be fully suppressed, proving mere presence of white
  // (even elsewhere in a scene that also contains green) is not enough.
  const int size = 40;
  cv::Mat bgr(size, size, CV_8UC3, kBlackBgr);
  bgr(cv::Rect(0, 20, size, 20)) = kGreenBgr;
  bgr(cv::Rect(10, 2, 20, 10)) = kWhiteBgr;

  auto params = make_default_params();
  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, params, white_mask, green_mask);
  auto field_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  auto line_mask = compute_line_mask(white_mask, field_mask, green_mask, params.morph_kernel_size, params.min_contour_area);

  EXPECT_EQ(cv::countNonZero(line_mask), 0);
}

TEST(ComputeLineMask, IsolatedNoisePixel_RemovedByOpening) {
  const int size = 40;
  cv::Mat bgr(size, size, CV_8UC3, kGreenBgr);
  // A single-pixel white speck inside the (fully green, hence fully
  // hull-covered) field -- should be removed by morphological opening.
  bgr.at<cv::Vec3b>(20, 20) = cv::Vec3b(255, 255, 255);

  auto params = make_default_params();
  params.morph_kernel_size = 3;  // enable opening
  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, params, white_mask, green_mask);
  auto field_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  auto line_mask = compute_line_mask(white_mask, field_mask, green_mask, params.morph_kernel_size, params.min_contour_area);

  EXPECT_EQ(cv::countNonZero(line_mask), 0);
}

TEST(ComputeLineMask, SmallContour_RemovedWhenMinAreaSet) {
  const int size = 40;
  cv::Mat bgr(size, size, CV_8UC3, kGreenBgr);
  // A small 2x2 white blob inside the field -- survives opening (kernel=1)
  // but should be removed once min_contour_area is set high enough.
  bgr(cv::Rect(19, 19, 2, 2)) = kWhiteBgr;

  auto params = make_default_params();
  params.morph_kernel_size = 1;  // no opening
  params.min_contour_area = 100;  // bigger than the 2x2=4 px^2 blob

  cv::Mat white_mask, green_mask;
  compute_white_green_masks(bgr, params, white_mask, green_mask);
  auto field_mask = compute_field_hull_mask(green_mask, /*border_margin_px=*/0);
  auto line_mask = compute_line_mask(white_mask, field_mask, green_mask, params.morph_kernel_size, params.min_contour_area);

  EXPECT_EQ(cv::countNonZero(line_mask), 0);
}

TEST(ComputeLineMask, PixelClassifiedAsBothWhiteAndGreen_Excluded) {
  // Simulates overlapping/loosely-configured HSV bounds where a bright turf
  // highlight satisfies both the white and green thresholds. Even though
  // it's "white" and inside the field hull, it must not be treated as a
  // line pixel -- it's still classified as field/green, and the field can
  // never be a line.
  const int size = 10;
  cv::Mat white_mask = cv::Mat::zeros(size, size, CV_8UC1);
  cv::Mat green_mask = cv::Mat::zeros(size, size, CV_8UC1);
  cv::Mat field_mask(size, size, CV_8UC1, cv::Scalar(255));

  white_mask.at<uint8_t>(5, 5) = 255;
  green_mask.at<uint8_t>(5, 5) = 255;  // same pixel also classified as green

  auto line_mask = compute_line_mask(white_mask, field_mask, green_mask, /*morph_kernel_size=*/1, /*min_contour_area=*/0);

  EXPECT_EQ(line_mask.at<uint8_t>(5, 5), 0);
}

// ===========================================================================
// suppress_candidates
// ===========================================================================

TEST(SuppressCandidates, InflatesAndBlacksOutRegion) {
  cv::Mat mask(50, 50, CV_8UC1, cv::Scalar(255));
  std::vector<Candidate> candidates = {Candidate::from_x1y1x2y2(20, 20, 30, 30, 0.9f)};

  suppress_candidates(mask, candidates, /*margin_px=*/2);

  // Inside inflated box -> zeroed
  EXPECT_EQ(mask.at<uint8_t>(25, 25), 0);
  EXPECT_EQ(mask.at<uint8_t>(19, 19), 0);  // just inside the 2px margin
  // Outside inflated box -> untouched
  EXPECT_EQ(mask.at<uint8_t>(0, 0), 255);
  EXPECT_EQ(mask.at<uint8_t>(45, 45), 255);
}

TEST(SuppressCandidates, ClampsAtImageBorder_NoCrash) {
  cv::Mat mask(20, 20, CV_8UC1, cv::Scalar(255));
  // Candidate box mostly outside the image bounds.
  std::vector<Candidate> candidates = {Candidate::from_x1y1x2y2(-10, -10, 5, 5, 0.9f)};

  EXPECT_NO_THROW(suppress_candidates(mask, candidates, 5));
  EXPECT_EQ(mask.at<uint8_t>(0, 0), 0);
}

// ===========================================================================
// segment_lines (end-to-end)
// ===========================================================================

TEST(SegmentLines, EndToEnd_LineDetected_BackgroundAndCandidateSuppressed) {
  const int size = 60;
  cv::Mat bgr(size, size, CV_8UC3, kGreenBgr);

  // A field line: vertical white stripe -- a hole in the (fully green) field.
  bgr(cv::Rect(size / 2 - 1, 0, 2, size)) = kWhiteBgr;

  // A "robot" white patch, also inside the field hull (so it would otherwise
  // be detected as a line-like region) — must be suppressed via the
  // candidate list, not the hull check.
  bgr(cv::Rect(5, 5, 6, 6)) = kWhiteBgr;

  auto params = make_default_params();
  std::vector<Candidate> suppress_list = {Candidate::from_x1y1x2y2(5, 5, 11, 11, 0.9f)};

  cv::Mat mask = segment_lines(bgr, params, suppress_list, /*suppress_margin_px=*/2);

  // Line region still present.
  EXPECT_GT(cv::countNonZero(mask(cv::Rect(size / 2 - 1, 5, 2, size - 10))), 0);
  // Suppressed candidate region is fully zeroed.
  EXPECT_EQ(cv::countNonZero(mask(cv::Rect(3, 3, 10, 10))), 0);
}
