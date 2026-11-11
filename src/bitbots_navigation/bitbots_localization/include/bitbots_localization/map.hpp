//
// Created by judith on 08.03.19.
//

#ifndef BITBOTS_LOCALIZATION_MAP_H
#define BITBOTS_LOCALIZATION_MAP_H

#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/tools.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gm = geometry_msgs;
namespace bl = bitbots_localization;

namespace bitbots_localization {

struct FieldDimensions {
  double x = 0;        // in m, x is the length of the field from goal to goal
  double y = 0;        // in m, y is the width of the field
  double padding = 0;  // in m, padding is the distance from the field lines to the field boundary
};

/**
 * @class Map
 * @brief Stores a map for a measurement class (e.g. a map of the lines)
 */
class Map {
 public:
  /**
   * @param map occupancy grid of the map in the map frame (e.g. as published by the parameter blackboard).
   * @param out_of_map_value value used for padding the out of field area.
   * @throws std::invalid_argument if the grid is malformed, rotated or contains values outside of [0, 100]
   */
  explicit Map(const nav_msgs::msg::OccupancyGrid& map, const double out_of_map_value);

  std::vector<double> provideRating(const RobotState& state,
                                    const std::vector<std::pair<double, double>>& observations);

  double get_occupancy(double x, double y);

  CartesianCoordinates getObservationCoordinatesInMapFrame(PolarCoordinates observation, double stateX, double stateY,
                                                           double stateT);

 private:
  // Occupancy values of the grid cells in percent, rows correspond to the y-axis and columns to the x-axis
  cv::Mat occupancy_;
  // Size of a grid cell in m
  double resolution_;
  // Position of the first grid cell in the map frame in m
  double origin_x_;
  double origin_y_;
  double out_of_map_value_;
};
};  // namespace bitbots_localization
#endif  // BITBOTS_LOCALIZATION_MAP_H
