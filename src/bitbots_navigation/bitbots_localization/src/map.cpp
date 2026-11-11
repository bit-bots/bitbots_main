//
// Created by judith on 08.03.19.
//

#include <bitbots_localization/map.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bitbots_localization {

namespace {
constexpr int kMaxOccupancy = 100;
constexpr double kMaxOriginRotation = 1e-6;
}  // namespace

Map::Map(const nav_msgs::msg::OccupancyGrid& map, const double out_of_map_value) {
  // Set config
  out_of_map_value_ = out_of_map_value;

  // Validate the grid, as we rely on its layout and value range when rating observations
  const auto& info = map.info;
  if (info.resolution <= 0.0) {
    throw std::invalid_argument("Field map has a non-positive resolution");
  }
  if (map.data.size() != static_cast<size_t>(info.width) * info.height) {
    std::stringstream message;
    message << "Field map contains " << map.data.size() << " cells, but its size is " << info.width << "x"
            << info.height;
    throw std::invalid_argument(message.str());
  }
  tf2::Quaternion origin_rotation;
  tf2::fromMsg(info.origin.orientation, origin_rotation);
  if (origin_rotation.angleShortestPath(tf2::Quaternion::getIdentity()) > kMaxOriginRotation) {
    throw std::invalid_argument("Field map has a rotated origin, which is not supported");
  }
  for (const int8_t value : map.data) {
    if (value < 0 || value > kMaxOccupancy) {
      std::stringstream message;
      message << "Field map contains the occupancy value " << static_cast<int>(value) << ", but only values within [0, "
              << kMaxOccupancy << "] are supported";
      throw std::invalid_argument(message.str());
    }
  }

  resolution_ = info.resolution;
  origin_x_ = info.origin.position.x;
  origin_y_ = info.origin.position.y;
  // Copy the data, as the message does not outlive this map
  occupancy_ = cv::Mat(info.height, info.width, CV_8U, const_cast<int8_t*>(map.data.data())).clone();
}

double Map::get_occupancy(double x, double y) {
  // Get the grid cell that contains the position
  const int column = std::floor((x - origin_x_) / resolution_);
  const int row = std::floor((y - origin_y_) / resolution_);

  double occupancy = out_of_map_value_;  // punish points outside the map

  if (column < occupancy_.cols && column >= 0 && row < occupancy_.rows && row >= 0) {
    occupancy = occupancy_.at<uchar>(row, column);
  }
  return occupancy / kMaxOccupancy;
}

std::vector<double> Map::provideRating(const RobotState& state,
                                       const std::vector<std::pair<double, double>>& observations) {
  std::vector<double> rating;
  for (const std::pair<double, double>& observation : observations) {
    // lines are in polar form!
    const PolarCoordinates observation_polar{observation.first, observation.second};

    // get rating per line
    const CartesianCoordinates line_relative =
        getObservationCoordinatesInMapFrame(observation_polar, state.getXPos(), state.getYPos(), state.getTheta());
    double occupancy = get_occupancy(line_relative.x, line_relative.y);

    rating.push_back(occupancy);
  }
  return rating;
}

CartesianCoordinates Map::getObservationCoordinatesInMapFrame(PolarCoordinates observation, double stateX,
                                                              double stateY, double stateT) {
  // queries the Cartesian metric map coordinates for a given observation (in polar coordinates)
  // taken relative to a given state (in Cartesian coordinates)
  // Input: Observation coordinates in polar coordinates, state coordinates in Cartesian coordinates
  // Output: Observation coordinates in Cartesian coordinates in the map frame

  // add theta and convert back to cartesian
  const CartesianCoordinates observation_with_theta =
      polarToCartesian({observation.angle + stateT, observation.radius});

  // add to particle
  return {stateX + observation_with_theta.x, stateY + observation_with_theta.y};
}
}  // namespace bitbots_localization
