#include <bitbots_localization/map.hpp>
#include <cmath>
#include <iostream>

int main() {
  bitbots_localization::Map map("hsl_kid", "lines.png", 25.0);
  map.map = cv::Mat(2, 256, CV_8UC1);
  for (int x = 0; x < map.map.cols; ++x) {
    map.map.at<uchar>(0, x) = x;
    map.map.at<uchar>(1, x) = x;
  }
  const auto grid = map.get_map_msg("map");
  for (int x = 0; x < map.map.cols; ++x) {
    const double score = map.get_occupancy((x - 128) / 100.0, 0.0);
    const double expected = 1.0 - x / 255.0;
    if (std::abs(score - expected) > 1e-12 || grid.data[x] < 0 || grid.data[x] > 100 ||
        std::abs(grid.data[x] / 100.0 - score) > 0.0051) {
      std::cerr << "Incorrect score or occupancy grid value for pixel " << x << '\n';
      return 1;
    }
  }
  if (map.get_occupancy(10.0, 0.0) != 0.25) {
    std::cerr << "Out-of-map percentage changed\n";
    return 1;
  }
  return 0;
}
