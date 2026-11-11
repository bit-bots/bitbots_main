#include <gtest/gtest.h>

#include <bitbots_localization/map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <stdexcept>
#include <vector>

namespace {
constexpr double kResolution = 0.5;
constexpr double kOutOfMapValue = 42.0;

// Creates a grid that is centered on the origin of the map frame
nav_msgs::msg::OccupancyGrid create_grid(uint32_t width, uint32_t height, const std::vector<int8_t>& data) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = kResolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = -(width / 2.0) * kResolution;
  grid.info.origin.position.y = -(height / 2.0) * kResolution;
  grid.info.origin.orientation.w = 1.0;
  grid.data = data;
  return grid;
}
}  // namespace

TEST(MapTest, rejects_unknown_cells) {
  EXPECT_THROW(bitbots_localization::Map(create_grid(2, 1, {0, -1}), kOutOfMapValue), std::invalid_argument);
}

TEST(MapTest, rejects_values_beyond_expected_range) {
  EXPECT_THROW(bitbots_localization::Map(create_grid(2, 1, {0, 101}), kOutOfMapValue), std::invalid_argument);
}

TEST(MapTest, rejects_size_mismatch) {
  EXPECT_THROW(bitbots_localization::Map(create_grid(2, 2, {0, 0, 0}), kOutOfMapValue), std::invalid_argument);
}

TEST(MapTest, rejects_rotated_origin) {
  auto grid = create_grid(2, 1, {0, 0});
  grid.info.origin.orientation.z = 1.0;
  grid.info.origin.orientation.w = 0.0;
  EXPECT_THROW(bitbots_localization::Map(grid, kOutOfMapValue), std::invalid_argument);
}

TEST(MapTest, looks_up_occupancy_of_grid_cells) {
  // Rows correspond to the y-axis, columns to the x-axis
  const std::vector<int8_t> data = {10, 20, 30, 40, 50, 60, 70, 80, 90};
  bitbots_localization::Map map(create_grid(3, 3, data), kOutOfMapValue);

  // Center of the grid
  EXPECT_DOUBLE_EQ(map.get_occupancy(0.0, 0.0), data[4] / 100.0);
  // One cell in positive x and y direction
  EXPECT_DOUBLE_EQ(map.get_occupancy(kResolution, 0.0), data[5] / 100.0);
  EXPECT_DOUBLE_EQ(map.get_occupancy(0.0, kResolution), data[7] / 100.0);
}

TEST(MapTest, uses_out_of_map_value_outside_of_grid) {
  bitbots_localization::Map map(create_grid(2, 1, {0, 0}), kOutOfMapValue);

  EXPECT_DOUBLE_EQ(map.get_occupancy(100.0, 0.0), kOutOfMapValue / 100.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
