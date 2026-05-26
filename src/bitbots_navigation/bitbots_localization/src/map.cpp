//
// Created by judith on 08.03.19.
//

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <bitbots_localization/map.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace bitbots_localization {

Map::Map(const std::string& name, const std::string& type, const double out_of_map_value) {
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
  return occupancy / 100.0;
}

std::vector<double> Map::provideRating(const RobotState& state,
                                       const std::vector<std::pair<double, double>>& observations) {
  std::vector<double> rating;
  for (const std::pair<double, double>& observation : observations) {
    // lines are in polar form!
    double line_x, line_y;

    // get rating per line
    getObservationCoordinatesInMapFrame(observation.first, observation.second, state.getXPos(), state.getYPos(),
                                        state.getTheta(), line_x, line_y);
    double occupancy = get_occupancy(line_x, line_y);

    rating.push_back(occupancy);
  }
  return rating;
}

void Map::getObservationCoordinatesInMapFrame(double obs_angle, double obs_radius, double stateX, double stateY,
                                              double stateT, double& result_x, double& result_y) {
  // queries the Cartesian metric map coordinates for a given observation (in polar coordinates)
  // taken relative to a given state (in Cartesian coordinates)
  // Input: Observation coordinates in polar coordinates, state coordinates in Cartesian coordinates
  // Output: Observation coordinates in Cartesian coordinates in the map frame

  // add theta and convert back to cartesian
  double cart_x, cart_y;
  polarToCartesian(obs_angle + stateT, obs_radius, cart_x, cart_y);

  // add to particle
  result_x = stateX + cart_x;
  result_y = stateY + cart_y;
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
