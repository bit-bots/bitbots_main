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
 * @brief Stores a map for a messurement class (e.g. a map of the lines)
 */
class Map {
 public:
  /**
   * @param name of the environment. (E.g. webots)
   * @param type of the map. (E.g. lines)
   * @param out_of_map_value value used for padding the out of field area.
   */
  explicit Map(const std::string& name, const std::string& type, const double out_of_map_value);

  cv::Mat map;

  std::vector<double> provideRating(const RobotState& state,
                                    const std::vector<std::pair<double, double>>& observations);

  double get_occupancy(double x, double y);

  std::pair<double, double> observationRelative(std::pair<double, double> observation, double stateX, double stateY,
                                                double stateT);

  nav_msgs::msg::OccupancyGrid get_map_msg(std::string frame_id, int threshold = -1);

 private:
  double out_of_map_value_ = 0;
  double max_value_ = 1;
};
};      // namespace bitbots_localization
#endif  // BITBOTS_LOCALIZATION_MAP_H
