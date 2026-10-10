#include <gtest/gtest.h>

#include <bitbots_localization/map.hpp>
#include <opencv2/core.hpp>

TEST(MapScalingTest, scales_full_range_map_to_expected_range) {
  uchar data[] = {0, 51, 153, 204, 12, 25, 128, 255};
  cv::Mat map(2, 4, CV_8U, data);

  bitbots_localization::scale_map_to_expected_range(map);

  double min_intensity = 0.0;
  double max_intensity = 0.0;
  cv::minMaxLoc(map, &min_intensity, &max_intensity);
  EXPECT_DOUBLE_EQ(min_intensity, 0.0);
  EXPECT_DOUBLE_EQ(max_intensity, 100.0);
  EXPECT_EQ(map.at<uchar>(0, 1), 20);
  EXPECT_EQ(map.at<uchar>(1, 2), 50);
}

TEST(MapScalingTest, keeps_maps_within_expected_range_unchanged) {
  uchar data[] = {0, 33, 66, 100};
  cv::Mat map(1, 4, CV_8U, data);
  const cv::Mat expected = map.clone();

  bitbots_localization::scale_map_to_expected_range(map);

  EXPECT_EQ(0, cv::countNonZero(expected != map));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
