#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include "bitbots_vision/candidate.hpp"
#include "bitbots_vision/yoeo_processing.hpp"

using namespace bitbots_vision::processing;
using bitbots_vision::Candidate;

// ===========================================================================
// preprocess_image
// ===========================================================================

TEST(PreprocessImage, OutputSizeIsNetHxNetW)
{
  cv::Mat bgr(100, 100, CV_8UC3, cv::Scalar(128, 64, 32));
  PreprocessInfo info;
  auto tensor = preprocess_image(bgr, 416, 416, info);
  EXPECT_EQ(static_cast<int>(tensor.size()), 3 * 416 * 416);
}

TEST(PreprocessImage, InfoFields_SquareImage)
{
  cv::Mat bgr(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
  PreprocessInfo info;
  preprocess_image(bgr, 416, 416, info);

  EXPECT_EQ(info.orig_h, 100);
  EXPECT_EQ(info.orig_w, 100);
  EXPECT_EQ(info.max_dim, 100);
  EXPECT_EQ(info.pad_top, 0);
  EXPECT_EQ(info.pad_bottom, 0);
  EXPECT_EQ(info.pad_left, 0);
  EXPECT_EQ(info.pad_right, 0);
  EXPECT_EQ(info.net_h, 416);
  EXPECT_EQ(info.net_w, 416);
}

TEST(PreprocessImage, InfoFields_WideImage_PadsTopBottom)
{
  // 100 tall × 200 wide → pad top and bottom
  cv::Mat bgr(100, 200, CV_8UC3, cv::Scalar(0, 0, 0));
  PreprocessInfo info;
  preprocess_image(bgr, 416, 416, info);

  EXPECT_EQ(info.orig_h, 100);
  EXPECT_EQ(info.orig_w, 200);
  EXPECT_EQ(info.max_dim, 200);
  EXPECT_EQ(info.pad_top, 50);
  EXPECT_EQ(info.pad_bottom, 50);
  EXPECT_EQ(info.pad_left, 0);
  EXPECT_EQ(info.pad_right, 0);
}

TEST(PreprocessImage, InfoFields_TallImage_PadsLeftRight)
{
  // 200 tall × 100 wide → pad left and right
  cv::Mat bgr(200, 100, CV_8UC3, cv::Scalar(0, 0, 0));
  PreprocessInfo info;
  preprocess_image(bgr, 416, 416, info);

  EXPECT_EQ(info.pad_top, 0);
  EXPECT_EQ(info.pad_bottom, 0);
  EXPECT_EQ(info.pad_left, 50);
  EXPECT_EQ(info.pad_right, 50);
}

TEST(PreprocessImage, InfoFields_OddDifference_PaddingSumIsCorrect)
{
  // 100 tall × 101 wide → difference 1, top=0, bottom=1
  cv::Mat bgr(100, 101, CV_8UC3, cv::Scalar(0, 0, 0));
  PreprocessInfo info;
  preprocess_image(bgr, 416, 416, info);

  EXPECT_EQ(info.pad_top + info.pad_bottom, 1);
  EXPECT_EQ(info.pad_left, 0);
  EXPECT_EQ(info.pad_right, 0);
}

TEST(PreprocessImage, Normalization_WhiteImage)
{
  cv::Mat bgr(4, 4, CV_8UC3, cv::Scalar(255, 255, 255));
  PreprocessInfo info;
  auto tensor = preprocess_image(bgr, 4, 4, info);

  for (float v : tensor) {
    EXPECT_NEAR(v, 1.0f, 1e-4f);
  }
}

TEST(PreprocessImage, Normalization_BlackImage)
{
  cv::Mat bgr(4, 4, CV_8UC3, cv::Scalar(0, 0, 0));
  PreprocessInfo info;
  auto tensor = preprocess_image(bgr, 4, 4, info);

  for (float v : tensor) {
    EXPECT_NEAR(v, 0.0f, 1e-4f);
  }
}

TEST(PreprocessImage, Normalization_ValuesInRange)
{
  cv::Mat bgr(50, 50, CV_8UC3);
  cv::randu(bgr, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
  PreprocessInfo info;
  auto tensor = preprocess_image(bgr, 32, 32, info);

  for (float v : tensor) {
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
  }
}

TEST(PreprocessImage, CHW_Layout)
{
  // Create a BGR image where channel B=0 G=128 R=255 for all pixels
  cv::Mat bgr(2, 2, CV_8UC3, cv::Scalar(0, 128, 255));  // B=0, G=128, R=255
  PreprocessInfo info;
  auto tensor = preprocess_image(bgr, 2, 2, info);
  // After BGR→RGB: R(255)→first channel, G(128)→second, B(0)→third
  // tensor layout: [C0(R), C1(G), C2(B)]
  // C0 should be ~1.0, C1 ~0.5, C2 ~0.0
  ASSERT_EQ(static_cast<int>(tensor.size()), 3 * 4);
  for (int px = 0; px < 4; ++px) {
    EXPECT_NEAR(tensor[0 * 4 + px], 1.0f, 0.01f);   // R channel
    EXPECT_NEAR(tensor[1 * 4 + px], 128.f / 255.f, 0.01f);  // G channel
    EXPECT_NEAR(tensor[2 * 4 + px], 0.0f, 0.01f);   // B channel
  }
}

// ===========================================================================
// nms_boxes
// ===========================================================================

TEST(NmsBoxes, SingleBox_AlwaysKept)
{
  std::vector<cv::Rect2d> boxes = {{10, 10, 20, 20}};
  std::vector<float> scores = {0.9f};
  std::vector<int> class_ids = {0};

  auto keep = nms_boxes(boxes, scores, class_ids, {}, 0.5f);
  EXPECT_EQ(keep.size(), 1u);
}

TEST(NmsBoxes, NonOverlapping_BothKept)
{
  // Two boxes far apart
  std::vector<cv::Rect2d> boxes = {{0, 0, 10, 10}, {500, 500, 10, 10}};
  std::vector<float> scores = {0.9f, 0.8f};
  std::vector<int> class_ids = {0, 0};

  auto keep = nms_boxes(boxes, scores, class_ids, {}, 0.5f);
  EXPECT_EQ(keep.size(), 2u);
}

TEST(NmsBoxes, IdenticalBoxes_SameClass_KeepsHighestScore)
{
  std::vector<cv::Rect2d> boxes = {{0, 0, 50, 50}, {0, 0, 50, 50}};
  std::vector<float> scores = {0.9f, 0.5f};
  std::vector<int> class_ids = {0, 0};

  auto keep = nms_boxes(boxes, scores, class_ids, {}, 0.5f);

  ASSERT_EQ(keep.size(), 1u);
  EXPECT_EQ(keep[0], 0);  // index 0 has the higher score
}

TEST(NmsBoxes, IdenticalBoxes_DifferentClasses_BothKept)
{
  // Same position, different class — should not suppress each other
  std::vector<cv::Rect2d> boxes = {{0, 0, 50, 50}, {0, 0, 50, 50}};
  std::vector<float> scores = {0.9f, 0.8f};
  std::vector<int> class_ids = {0, 1};  // different classes

  auto keep = nms_boxes(boxes, scores, class_ids, {}, 0.5f);
  EXPECT_EQ(keep.size(), 2u);
}

TEST(NmsBoxes, RobotClasses_CrossClassSuppression)
{
  // Two robot classes at identical position — cross-suppressed to one
  std::vector<cv::Rect2d> boxes = {{0, 0, 50, 50}, {0, 0, 50, 50}};
  std::vector<float> scores = {0.9f, 0.8f};
  std::vector<int> class_ids = {1, 2};  // both are robot classes
  std::vector<int> robot_class_ids = {1, 2};

  auto keep = nms_boxes(boxes, scores, class_ids, robot_class_ids, 0.5f);
  EXPECT_EQ(keep.size(), 1u);
}

TEST(NmsBoxes, RobotClasses_NonRobotNotAffected)
{
  // One robot and one non-robot at identical position — only robot class is merged
  std::vector<cv::Rect2d> boxes = {
    {0, 0, 50, 50},   // ball (class 0)
    {0, 0, 50, 50},   // robot_blue (class 1)
    {0, 0, 50, 50},   // robot_red (class 2)
  };
  std::vector<float> scores = {0.9f, 0.8f, 0.7f};
  std::vector<int> class_ids = {0, 1, 2};
  std::vector<int> robot_class_ids = {1, 2};

  auto keep = nms_boxes(boxes, scores, class_ids, robot_class_ids, 0.5f);
  // ball and one robot (highest score) should be kept — the other robot suppressed
  EXPECT_EQ(keep.size(), 2u);
}

TEST(NmsBoxes, MaxDetections_Respected)
{
  // 20 non-overlapping boxes, max_detections=5
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < 20; ++i) {
    boxes.push_back({static_cast<double>(i * 100), 0, 10, 10});
    scores.push_back(0.5f + i * 0.01f);
    class_ids.push_back(0);
  }

  auto keep = nms_boxes(boxes, scores, class_ids, {}, 0.5f, 5);
  EXPECT_LE(static_cast<int>(keep.size()), 5);
}

// ===========================================================================
// postprocess_detections
// ===========================================================================

// Helper: build raw detection output [num_boxes, 5 + num_classes]
// Each box row: [x_c, y_c, w, h, obj_conf, class_prob_0, class_prob_1, ...]
static std::vector<float> make_det_row(
  float xc, float yc, float w, float h,
  float obj_conf,
  std::initializer_list<float> class_probs)
{
  std::vector<float> row = {xc, yc, w, h, obj_conf};
  for (float p : class_probs) {
    row.push_back(p);
  }
  return row;
}

TEST(PostprocessDetections, Empty_ReturnsEmptyMap)
{
  PreprocessInfo info{100, 100, 100, 0, 0, 0, 0, 416, 416};
  auto result = postprocess_detections(nullptr, 0, 7, {"ball", "robot"}, {1}, 0.5f, 0.4f, info);
  EXPECT_TRUE(result.empty());
}

TEST(PostprocessDetections, BelowObjConfThreshold_Filtered)
{
  auto row = make_det_row(208, 208, 50, 50, 0.1f, {1.0f, 0.0f});  // obj_conf=0.1
  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);
  EXPECT_TRUE(result.empty());
}

TEST(PostprocessDetections, BelowCombinedScoreThreshold_Filtered)
{
  // obj_conf=0.8, class_prob=0.3 → combined=0.24 < 0.5
  auto row = make_det_row(208, 208, 50, 50, 0.8f, {0.3f, 0.0f});
  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);
  EXPECT_TRUE(result.empty());
}

TEST(PostprocessDetections, SingleDetection_CorrectClass)
{
  // Ball detection: obj_conf=1.0, class_prob ball=1.0
  auto row = make_det_row(208, 208, 50, 50, 1.0f, {1.0f, 0.0f});
  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_TRUE(result.count("ball") > 0);
  EXPECT_EQ(result.at("ball").size(), 1u);
  EXPECT_TRUE(result.count("robot") == 0 || result.at("robot").empty());
}

TEST(PostprocessDetections, SingleDetection_RobotClass)
{
  // Robot detection: class index 1 (robot)
  auto row = make_det_row(100, 100, 40, 80, 1.0f, {0.0f, 1.0f});
  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_TRUE(result.count("robot") > 0);
  EXPECT_EQ(result.at("robot").size(), 1u);
}

TEST(PostprocessDetections, CoordinateRescaling_SquareImage)
{
  // 832x832 original, 416x416 network → scale = 2.0
  // Ball at x_c=208, y_c=208, w=50, h=50 in network space
  // → x1=(208-25)*2=366, y1=366, x2=(208+25)*2=466, y2=466
  // → center_x = 366 + 50 = 416, center_y = 416
  auto row = make_det_row(208.f, 208.f, 50.f, 50.f, 1.0f, {1.0f, 0.0f});
  PreprocessInfo info{832, 832, 832, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_TRUE(result.count("ball") > 0);
  const auto & c = result.at("ball")[0];
  EXPECT_NEAR(c.center_x(), 416, 5);
  EXPECT_NEAR(c.center_y(), 416, 5);
  EXPECT_NEAR(c.width, 100, 5);
  EXPECT_NEAR(c.height, 100, 5);
}

TEST(PostprocessDetections, CoordinateRescaling_WithPadding)
{
  // 100x200 original (wide), padded → max_dim=200, pad_top=50, pad_bottom=50
  // Network 416x416. Ball centered at (208, 208) in network space.
  // scale = 200 / 416 ≈ 0.481
  // y1 = (208 - 25) * scale - pad_top = 183 * 0.481 - 50 ≈ 88 - 50 = 38
  // y2 = (208 + 25) * scale - pad_top = 233 * 0.481 - 50 ≈ 112 - 50 = 62
  // center_y ≈ (38 + 62) / 2 = 50 = orig_h / 2 ✓
  auto row = make_det_row(208.f, 208.f, 20.f, 20.f, 1.0f, {1.0f, 0.0f});
  PreprocessInfo info{100, 200, 200, 50, 50, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_TRUE(result.count("ball") > 0);
  const auto & c = result.at("ball")[0];
  // Center should be approximately at the middle of the original image
  EXPECT_NEAR(c.center_x(), 100, 10);   // orig_w center
  EXPECT_NEAR(c.center_y(), 50, 10);    // orig_h center
}

TEST(PostprocessDetections, NMS_SuppressesOverlappingBoxes)
{
  // Two nearly identical ball detections → NMS keeps only one
  std::vector<float> data;
  auto row1 = make_det_row(200.f, 200.f, 50.f, 50.f, 1.0f, {1.0f, 0.0f});
  auto row2 = make_det_row(201.f, 201.f, 50.f, 50.f, 0.9f, {1.0f, 0.0f});
  data.insert(data.end(), row1.begin(), row1.end());
  data.insert(data.end(), row2.begin(), row2.end());

  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};
  auto result = postprocess_detections(
    data.data(), 2, static_cast<int64_t>(row1.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_TRUE(result.count("ball") > 0);
  EXPECT_EQ(result.at("ball").size(), 1u);
}

TEST(PostprocessDetections, NMS_KeepsSeparateClasses)
{
  // A ball and a robot at identical position → different classes, both kept
  std::vector<float> data;
  auto row1 = make_det_row(200.f, 200.f, 50.f, 50.f, 1.0f, {1.0f, 0.0f});  // ball
  auto row2 = make_det_row(200.f, 200.f, 50.f, 50.f, 0.9f, {0.0f, 1.0f});  // robot
  data.insert(data.end(), row1.begin(), row1.end());
  data.insert(data.end(), row2.begin(), row2.end());

  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};
  auto result = postprocess_detections(
    data.data(), 2, static_cast<int64_t>(row1.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  EXPECT_EQ(result["ball"].size(), 1u);
  EXPECT_EQ(result["robot"].size(), 1u);
}

TEST(PostprocessDetections, ConfidenceStoredInCandidate)
{
  auto row = make_det_row(208.f, 208.f, 50.f, 50.f, 1.0f, {0.8f, 0.0f});
  PreprocessInfo info{416, 416, 416, 0, 0, 0, 0, 416, 416};

  auto result = postprocess_detections(
    row.data(), 1, static_cast<int64_t>(row.size()), {"ball", "robot"}, {1}, 0.5f, 0.4f, info);

  ASSERT_FALSE(result["ball"].empty());
  EXPECT_NEAR(result.at("ball")[0].rating, 0.8f, 0.01f);
}

// ===========================================================================
// postprocess_segmentation
// ===========================================================================

TEST(PostprocessSegmentation, MaskCountMatchesClassCount)
{
  std::vector<float> data(16 * 16, 0.f);
  PreprocessInfo info{16, 16, 16, 0, 0, 0, 0, 16, 16};

  auto result = postprocess_segmentation(
    data.data(), 16, 16, {"background", "field", "lines"}, info);

  EXPECT_EQ(result.size(), 3u);
  EXPECT_TRUE(result.count("background") > 0);
  EXPECT_TRUE(result.count("field") > 0);
  EXPECT_TRUE(result.count("lines") > 0);
}

TEST(PostprocessSegmentation, MaskSizeMatchesOriginal)
{
  // 150-tall × 100-wide image (tall): pad_left=25, pad_right=25
  std::vector<float> data(8 * 8, 0.f);
  PreprocessInfo info{150, 100, 150, 0, 0, 25, 25, 8, 8};

  auto result = postprocess_segmentation(
    data.data(), 8, 8, {"background", "field"}, info);

  ASSERT_TRUE(result.count("background") > 0);
  EXPECT_EQ(result.at("background").rows, 150);
  EXPECT_EQ(result.at("background").cols, 100);
}

TEST(PostprocessSegmentation, AllClass0_Class0MaskAllActive)
{
  // All pixels → class 0 (background)
  const int H = 8, W = 8;
  std::vector<float> data(H * W, 0.f);
  PreprocessInfo info{H, W, W, 0, 0, 0, 0, H, W};

  auto result = postprocess_segmentation(
    data.data(), H, W, {"background", "field"}, info);

  ASSERT_TRUE(result.count("background") > 0);
  ASSERT_TRUE(result.count("field") > 0);

  // background mask should be non-zero
  double min_bg, max_bg;
  cv::minMaxLoc(result.at("background"), &min_bg, &max_bg);
  EXPECT_GT(max_bg, 0.0);

  // field mask should be all zero
  double max_field;
  cv::minMaxLoc(result.at("field"), nullptr, &max_field);
  EXPECT_EQ(max_field, 0.0);
}

TEST(PostprocessSegmentation, AllClass1_Class1MaskAllActive)
{
  const int H = 8, W = 8;
  std::vector<float> data(H * W, 1.f);  // all pixels → class 1
  PreprocessInfo info{H, W, W, 0, 0, 0, 0, H, W};

  auto result = postprocess_segmentation(
    data.data(), H, W, {"background", "field"}, info);

  // field (class 1) should be all active
  double max_field;
  cv::minMaxLoc(result.at("field"), nullptr, &max_field);
  EXPECT_GT(max_field, 0.0);

  // background (class 0) should be all zero
  double max_bg;
  cv::minMaxLoc(result.at("background"), nullptr, &max_bg);
  EXPECT_EQ(max_bg, 0.0);
}

TEST(PostprocessSegmentation, UnpaddingSquareOriginal_NoChange)
{
  // Square original → no padding, crop is identity
  const int H = 4, W = 4;
  std::vector<float> data(H * W, 0.f);
  PreprocessInfo info{H, W, W, 0, 0, 0, 0, H, W};

  auto result = postprocess_segmentation(data.data(), H, W, {"bg"}, info);
  ASSERT_TRUE(result.count("bg") > 0);
  EXPECT_EQ(result.at("bg").rows, H);
  EXPECT_EQ(result.at("bg").cols, W);
}

TEST(PostprocessSegmentation, UnpaddingWideOriginal_OutputHeightReduced)
{
  // 200×100 original (wide): max_dim=200, pad_top=50, pad_bottom=50
  // Result should be cropped to 100×200
  const int H = 8, W = 8;
  std::vector<float> data(H * W, 0.f);
  PreprocessInfo info{100, 200, 200, 50, 50, 0, 0, H, W};

  auto result = postprocess_segmentation(data.data(), H, W, {"bg"}, info);
  ASSERT_TRUE(result.count("bg") > 0);
  EXPECT_EQ(result.at("bg").rows, 100);
  EXPECT_EQ(result.at("bg").cols, 200);
}

TEST(PostprocessSegmentation, UnpaddingTallOriginal_OutputWidthReduced)
{
  // 100×200 original (tall): max_dim=200, pad_left=50, pad_right=50
  const int H = 8, W = 8;
  std::vector<float> data(H * W, 0.f);
  PreprocessInfo info{200, 100, 200, 0, 0, 50, 50, H, W};

  auto result = postprocess_segmentation(data.data(), H, W, {"bg"}, info);
  ASSERT_TRUE(result.count("bg") > 0);
  EXPECT_EQ(result.at("bg").rows, 200);
  EXPECT_EQ(result.at("bg").cols, 100);
}

TEST(PostprocessSegmentation, MaskTypeIsCV_8UC1)
{
  const int H = 4, W = 4;
  std::vector<float> data(H * W, 0.f);
  PreprocessInfo info{H, W, W, 0, 0, 0, 0, H, W};

  auto result = postprocess_segmentation(data.data(), H, W, {"bg"}, info);
  EXPECT_EQ(result.at("bg").type(), CV_8UC1);
}

TEST(PostprocessSegmentation, MixedClasses_MutuallyExclusiveMasks)
{
  // 4x4 map: left half class 0, right half class 1
  const int H = 4, W = 4;
  std::vector<float> data(H * W);
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      data[r * W + c] = (c < W / 2) ? 0.f : 1.f;
    }
  }
  PreprocessInfo info{H, W, W, 0, 0, 0, 0, H, W};

  auto result = postprocess_segmentation(data.data(), H, W, {"bg", "field"}, info);

  // bg active only in left half
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      if (c < W / 2) {
        EXPECT_GT(result.at("bg").at<uint8_t>(r, c), 0u)
          << "bg should be active at (" << r << "," << c << ")";
        EXPECT_EQ(result.at("field").at<uint8_t>(r, c), 0u)
          << "field should be inactive at (" << r << "," << c << ")";
      } else {
        EXPECT_EQ(result.at("bg").at<uint8_t>(r, c), 0u)
          << "bg should be inactive at (" << r << "," << c << ")";
        EXPECT_GT(result.at("field").at<uint8_t>(r, c), 0u)
          << "field should be active at (" << r << "," << c << ")";
      }
    }
  }
}
