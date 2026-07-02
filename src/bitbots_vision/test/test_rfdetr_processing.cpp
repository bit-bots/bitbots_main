#include <gtest/gtest.h>

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <cmath>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

using namespace bitbots_vision::processing;
using bitbots_vision::Candidate;

// ===========================================================================
// preprocess_image_rfdetr
// ===========================================================================

TEST(PreprocessImageRfdetr, OutputSizeIsNetHxNetW) {
  cv::Mat bgr(100, 100, CV_8UC3, cv::Scalar(128, 64, 32));
  auto tensor = preprocess_image_rfdetr(bgr, 384, 384);
  EXPECT_EQ(static_cast<int>(tensor.size()), 3 * 384 * 384);
}

TEST(PreprocessImageRfdetr, DirectResize_NonSquareInput_NoBlackBorders) {
  // A solid-colour non-square image should stay solid-colour after
  // preprocessing — unlike YOEO, RF-DETR uses a direct resize with no
  // aspect-ratio padding, so there must be no black (zero) border pixels.
  cv::Mat bgr(100, 200, CV_8UC3, cv::Scalar(200, 200, 200));
  auto tensor = preprocess_image_rfdetr(bgr, 32, 32);

  // A padded border would introduce pixels normalized towards the
  // BGR2RGB(0,0,0) ImageNet-normalized value, i.e. -mean/std, which is very
  // different (very negative) from the solid grey value's normalized result.
  // Every value should be close to the single expected normalized value per
  // channel since the whole image is solid colour.
  const float expected[3] = {
      (200.f / 255.f - 0.485f) / 0.229f,  // R
      (200.f / 255.f - 0.456f) / 0.224f,  // G
      (200.f / 255.f - 0.406f) / 0.225f,  // B
  };
  const int hw = 32 * 32;
  for (int c = 0; c < 3; ++c) {
    for (int i = 0; i < hw; ++i) {
      EXPECT_NEAR(tensor[c * hw + i], expected[c], 1e-2f);
    }
  }
}

TEST(PreprocessImageRfdetr, ImageNetNormalization_KnownColor) {
  // BGR = (0, 128, 255) -> RGB = (255, 128, 0)
  cv::Mat bgr(2, 2, CV_8UC3, cv::Scalar(0, 128, 255));
  auto tensor = preprocess_image_rfdetr(bgr, 2, 2);

  const float r = 255.f / 255.f;
  const float g = 128.f / 255.f;
  const float b = 0.f / 255.f;
  const float expected_r = (r - 0.485f) / 0.229f;
  const float expected_g = (g - 0.456f) / 0.224f;
  const float expected_b = (b - 0.406f) / 0.225f;

  ASSERT_EQ(static_cast<int>(tensor.size()), 3 * 4);
  for (int px = 0; px < 4; ++px) {
    EXPECT_NEAR(tensor[0 * 4 + px], expected_r, 1e-2f);
    EXPECT_NEAR(tensor[1 * 4 + px], expected_g, 1e-2f);
    EXPECT_NEAR(tensor[2 * 4 + px], expected_b, 1e-2f);
  }
}

// ===========================================================================
// postprocess_detections_rfdetr
// ===========================================================================

TEST(PostprocessDetectionsRfdetr, Empty_ReturnsEmptyMap) {
  auto result = postprocess_detections_rfdetr(nullptr, nullptr, 0, 4, {"ball", "unknown", "goalpost", "robot"}, {}, 0.5f,
                                              416, 416);
  EXPECT_TRUE(result.empty());
}

TEST(PostprocessDetectionsRfdetr, BelowConfThreshold_Filtered) {
  // Near-uniform logits -> softmax confidence close to 1/num_classes, well below 0.5
  std::vector<float> dets = {0.5f, 0.5f, 0.1f, 0.1f};
  std::vector<float> logits = {0.1f, 0.0f, 0.0f, 0.0f};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 1, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.9f, 416, 416);
  EXPECT_TRUE(result.empty());
}

TEST(PostprocessDetectionsRfdetr, SingleDetection_CorrectClass) {
  // Class 0 ("ball") dominant logit
  std::vector<float> dets = {0.5f, 0.5f, 0.1f, 0.1f};
  std::vector<float> logits = {10.f, -10.f, -10.f, -10.f};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 1, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.5f, 416, 416);

  ASSERT_TRUE(result.count("ball") > 0);
  EXPECT_EQ(result.at("ball").size(), 1u);
  EXPECT_TRUE(result.count("robot") == 0);
}

TEST(PostprocessDetectionsRfdetr, UnknownClass_NeverSpecialCased_JustAccumulates) {
  // Class 1 ("unknown") dominant — result should contain it under its own
  // key; the calling code simply never queries "unknown".
  std::vector<float> dets = {0.5f, 0.5f, 0.1f, 0.1f};
  std::vector<float> logits = {-10.f, 10.f, -10.f, -10.f};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 1, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.5f, 416, 416);

  ASSERT_TRUE(result.count("unknown") > 0);
  EXPECT_EQ(result.at("unknown").size(), 1u);
}

TEST(PostprocessDetectionsRfdetr, CoordinateDecode_UsesOriginalImageDims) {
  // cx=0.5, cy=0.5, w=0.25, h=0.5 normalized, orig image 800x400 (w x h)
  // x1 = (0.5 - 0.125) * 800 = 300, x2 = (0.5 + 0.125) * 800 = 500
  // y1 = (0.5 - 0.25) * 400 = 100,  y2 = (0.5 + 0.25) * 400 = 300
  std::vector<float> dets = {0.5f, 0.5f, 0.25f, 0.5f};
  std::vector<float> logits = {10.f, -10.f, -10.f, -10.f};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 1, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.5f,
                                              /*orig_h=*/400, /*orig_w=*/800);

  ASSERT_TRUE(result.count("ball") > 0);
  const auto& c = result.at("ball")[0];
  EXPECT_NEAR(c.x1, 300, 1);
  EXPECT_NEAR(c.y1, 100, 1);
  EXPECT_NEAR(c.x2(), 500, 1);
  EXPECT_NEAR(c.y2(), 300, 1);
}

TEST(PostprocessDetectionsRfdetr, NoNms_OverlappingSameClassDetections_BothKept) {
  // Two nearly-identical high-confidence ball detections — RF-DETR does no
  // NMS, so both must be kept (unlike the old YOEO pipeline).
  std::vector<float> dets = {
      0.5f, 0.5f, 0.2f, 0.2f,     //
      0.51f, 0.51f, 0.2f, 0.2f,  //
  };
  std::vector<float> logits = {
      10.f, -10.f, -10.f, -10.f,  //
      9.f, -10.f, -10.f, -10.f,   //
  };

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 2, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.5f, 416, 416);

  ASSERT_TRUE(result.count("ball") > 0);
  EXPECT_EQ(result.at("ball").size(), 2u);
}

TEST(PostprocessDetectionsRfdetr, ConfidenceStoredInCandidate) {
  std::vector<float> dets = {0.5f, 0.5f, 0.2f, 0.2f};
  // logits chosen so softmax(class 0) is a known, computable value
  std::vector<float> logits = {2.0f, 0.0f, 0.0f, 0.0f};
  const float e = std::exp(1.f);  // relative to subtracting max(2.0): exp(0)=1 for class0, exp(-2) for others*3
  const float exp0 = 1.f;
  const float exp_other = std::exp(-2.f);
  const float expected_conf = exp0 / (exp0 + 3 * exp_other);
  (void)e;

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 1, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, {}, 0.1f, 416, 416);

  ASSERT_TRUE(result.count("ball") > 0);
  EXPECT_NEAR(result.at("ball")[0].rating, expected_conf, 1e-4f);
}

TEST(PostprocessDetectionsRfdetr, PerClassThreshold_DifferentCutoffsPerClass) {
  // Both detections have the same confidence (~0.968), which is above the
  // "robot" threshold (0.9) but below the "ball" threshold (0.98) — so the
  // ball detection must be dropped while the robot detection is kept.
  std::vector<float> dets = {
      0.5f, 0.5f, 0.1f, 0.1f,  // row 0: ball
      0.5f, 0.5f, 0.1f, 0.1f,  // row 1: robot
  };
  // Other-class logits are 0 (not a large negative number) so the dominant
  // logit alone controls the resulting confidence: softmax = e^L / (e^L + 3).
  std::vector<float> logits = {
      4.5f, 0.f, 0.f, 0.f,  // dominant class 0 ("ball")
      0.f, 0.f, 0.f, 4.5f,  // dominant class 3 ("robot")
  };
  std::unordered_map<std::string, float> class_conf_thresholds = {{"ball", 0.98f}, {"robot", 0.9f}};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 2, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, class_conf_thresholds, 0.5f,
                                              416, 416);

  EXPECT_TRUE(result["ball"].empty());
  ASSERT_FALSE(result["robot"].empty());
  EXPECT_EQ(result.at("robot").size(), 1u);
}

TEST(PostprocessDetectionsRfdetr, PerClassThreshold_UnlistedClassUsesDefault) {
  // "goalpost" has no dedicated entry in class_conf_thresholds, so it must
  // fall back to default_conf_thresh (0.5 here). Confidence ~0.475 is
  // rejected, ~0.870 is kept.
  std::vector<float> dets = {
      0.5f, 0.5f, 0.1f, 0.1f,
      0.5f, 0.5f, 0.1f, 0.1f,
  };
  // Other-class logits are 0 so softmax = e^L / (e^L + 3), matching the
  // confidences noted above.
  std::vector<float> logits = {
      0.f, 0.f, 1.0f, 0.f,  // dominant class 2 ("goalpost"), conf ~0.475
      0.f, 0.f, 3.0f, 0.f,  // dominant class 2 ("goalpost"), conf ~0.870
  };
  std::unordered_map<std::string, float> class_conf_thresholds = {{"ball", 0.98f}, {"robot", 0.9f}};

  auto result = postprocess_detections_rfdetr(dets.data(), logits.data(), 2, 4,
                                              {"ball", "unknown", "goalpost", "robot"}, class_conf_thresholds, 0.5f,
                                              416, 416);

  ASSERT_TRUE(result.count("goalpost") > 0);
  EXPECT_EQ(result.at("goalpost").size(), 1u);
}
