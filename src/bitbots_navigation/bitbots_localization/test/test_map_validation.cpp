#include <gtest/gtest.h>

#include <bitbots_localization/map.hpp>
#include <opencv2/core.hpp>
#include <string>

TEST(MapValidationTest, throws_for_maps_beyond_expected_range) {
  uchar data[] = {0, 51, 153, 204, 12, 25, 128, 255};
  cv::Mat map(2, 4, CV_8U, data);

  EXPECT_THROW(bitbots_localization::validate_map_value_range(map, "lines.png"), std::invalid_argument);

  try {
    bitbots_localization::validate_map_value_range(map, "lines.png");
  } catch (const std::invalid_argument& e) {
    const std::string message = e.what();
    EXPECT_NE(message.find("lines.png"), std::string::npos);
    EXPECT_NE(message.find("255"), std::string::npos);
    EXPECT_NE(message.find("100"), std::string::npos);
  }
}

TEST(MapValidationTest, accepts_maps_within_expected_range) {
  uchar data[] = {0, 33, 66, 100};
  cv::Mat map(1, 4, CV_8U, data);

  EXPECT_NO_THROW(bitbots_localization::validate_map_value_range(map, "lines.png"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
