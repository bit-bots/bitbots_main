#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "bitbots_vision/model_config.hpp"

using bitbots_vision::ModelConfig;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Write a minimal model_config.yaml to a temp directory and return the dir.
static std::string make_temp_model_dir(const std::string& yaml_content) {
  // Use /tmp/<test-pid>-bitbots_vision_test as temp dir so we don't collide.
  const std::string dir = "/tmp/bitbots_vision_test_" + std::to_string(getpid());
  std::filesystem::create_directories(dir);

  std::ofstream f(dir + "/model_config.yaml");
  f << yaml_content;
  return dir;
}

// ---------------------------------------------------------------------------
// Basic loading
// ---------------------------------------------------------------------------

TEST(ModelConfig, DetectionClasses) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
    - goalpost
    - robot
segmentation:
  classes:
    - background
    - field
    - lines
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);

  ASSERT_EQ(cfg.detection_classes().size(), 3u);
  EXPECT_EQ(cfg.detection_classes()[0], "ball");
  EXPECT_EQ(cfg.detection_classes()[1], "goalpost");
  EXPECT_EQ(cfg.detection_classes()[2], "robot");
}

TEST(ModelConfig, SegmentationClasses) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
segmentation:
  classes:
    - background
    - field
    - lines
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);

  ASSERT_EQ(cfg.segmentation_classes().size(), 3u);
  EXPECT_EQ(cfg.segmentation_classes()[0], "background");
  EXPECT_EQ(cfg.segmentation_classes()[1], "field");
  EXPECT_EQ(cfg.segmentation_classes()[2], "lines");
}

// ---------------------------------------------------------------------------
// team_colors_provided
// ---------------------------------------------------------------------------

TEST(ModelConfig, TeamColorsProvidedFalse_WhenKeyAbsent) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  EXPECT_FALSE(cfg.team_colors_provided());
}

TEST(ModelConfig, TeamColorsProvidedTrue) {
  const std::string yaml = R"(
detection:
  classes:
    - robot_blue
    - robot_red
  team_colors: true
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  EXPECT_TRUE(cfg.team_colors_provided());
}

TEST(ModelConfig, TeamColorsProvidedFalse_WhenExplicitFalse) {
  const std::string yaml = R"(
detection:
  classes:
    - robot
  team_colors: false
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  EXPECT_FALSE(cfg.team_colors_provided());
}

// ---------------------------------------------------------------------------
// robot_class_ids
// ---------------------------------------------------------------------------

TEST(ModelConfig, RobotClassIds_SingleRobotClass) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
    - goalpost
    - robot
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  auto ids = cfg.robot_class_ids();

  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], 2);  // "robot" is at index 2
}

TEST(ModelConfig, RobotClassIds_MultipleRobotClasses) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
    - robot_blue
    - robot_red
    - robot_unknown
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  auto ids = cfg.robot_class_ids();

  ASSERT_EQ(ids.size(), 3u);
  EXPECT_EQ(ids[0], 1);
  EXPECT_EQ(ids[1], 2);
  EXPECT_EQ(ids[2], 3);
}

TEST(ModelConfig, RobotClassIds_NoRobotClass) {
  const std::string yaml = R"(
detection:
  classes:
    - ball
    - goalpost
segmentation:
  classes:
    - background
)";
  auto dir = make_temp_model_dir(yaml);
  auto cfg = ModelConfig::load_from(dir);
  EXPECT_TRUE(cfg.robot_class_ids().empty());
}

// ---------------------------------------------------------------------------
// Error handling
// ---------------------------------------------------------------------------

TEST(ModelConfig, ThrowsOnMissingFile) {
  EXPECT_THROW(ModelConfig::load_from("/nonexistent/path"), std::runtime_error);
}
