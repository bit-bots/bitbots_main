//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/map.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace bitbots_localization {

Map::Map(const std::string &name, const std::string &type, const double out_of_map_value) {
  // Set config
  out_of_map_value_ = out_of_map_value;
  // get package path
  std::string map_package_path = ament_index_cpp::get_package_share_directory("bitbots_parameter_blackboard");
  // make boost path
  fs::path map_path = fs::path("config/fields") / fs::path(name) / fs::path(type);
  // convert to absolute path
  fs::path absolute_map_path = fs::absolute(map_path, map_package_path);
  // load map
  map = cv::imread(absolute_map_path.string(), cv::IMREAD_GRAYSCALE);
  if (!map.data) {
    RCLCPP_ERROR(rclcpp::get_logger("bitbots_localization"), "No image data '%s'", map_path.c_str());
    return;
  }
}

double Map::get_occupancy(double x, double y) {
  // get dimensions of field
  int mapWidth = map.cols;
  int mapHeight = map.rows;

  // m to pixel (=cm)
  x = x * 100;
  y = y * 100;

  // ursprung in feldmitte
  x = std::round(x + mapWidth / 2.0);  // assuming lines are centered on map
  y = std::round(y + mapHeight / 2.0);

  double occupancy = out_of_map_value_;  // punish points outside the map

  if (x < mapWidth && x >= 0 && y < mapHeight && y >= 0) {
    occupancy = 100 - map.at<uchar>(y, x);
  }
  return occupancy;
}

std::vector<double> Map::provideRating(const RobotState &state,
                                       const std::vector<std::pair<double, double>> &observations) {
  std::vector<double> rating;
  for (const std::pair<double, double> &observation : observations) {
    // lines are in polar form!
    std::pair<double, double> lineRelative;

    // get rating per line
    lineRelative = observationRelative(observation, state.getXPos(), state.getYPos(), state.getTheta());
    double occupancy = get_occupancy(lineRelative.first, lineRelative.second);

    rating.push_back(occupancy);
  }
  return rating;
}

std::pair<double, double> Map::observationRelative(
    std::pair<double, double> observation, double stateX, double stateY,
    double stateT) {  // todo rename to a more correct name like observationonmap?
  // transformes observation from particle to map (assumes particle is correct)
  // input: obsservation relative in polar coordinates and particle
  // output: hypothetical observation on map

  // add theta and convert back to cartesian
  std::pair<double, double> observationWithTheta = polarToCartesian(observation.first + stateT, observation.second);

  // add to particle
  std::pair<double, double> observationRelative =
      std::make_pair(stateX + observationWithTheta.first, stateY + observationWithTheta.second);

  // alternativ:
  // Thrun 6.32, seite 169
  //  but both equivalent
  // double xGlobal = stateX + observation.second * (cos(stateT + observation.first));
  // double yGlobal = stateY + observation.second * (sin(stateT + observation.first));

  // std::pair<double, double> observationRelative = std::make_pair(xGlobal, yGlobal);

  return observationRelative;  // in cartesian
}

nav_msgs::msg::OccupancyGrid Map::get_map_msg(std::string frame_id, int threshold) {
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.frame_id = frame_id;
  map_msg.info.resolution = 0.01;
  map_msg.info.width = map.cols;
  map_msg.info.height = map.rows;
  map_msg.info.origin.position.x = -map.cols / 2.0 * map_msg.info.resolution;
  map_msg.info.origin.position.y = -map.rows / 2.0 * map_msg.info.resolution;
  map_msg.info.origin.position.z = 0;
  map_msg.info.origin.orientation.x = 0;
  map_msg.info.origin.orientation.y = 0;
  map_msg.info.origin.orientation.z = 0;
  map_msg.info.origin.orientation.w = 1;
  map_msg.data.resize(map.rows * map.cols);
  for (int i = 0; i < map.rows; i++) {
    for (int j = 0; j < map.cols; j++) {
      map_msg.data[i * map.cols + j] = 100 - map.at<uchar>(i, j);
    }
  }
  return map_msg;
}
}  // namespace bitbots_localization
