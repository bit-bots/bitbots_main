//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/map.h"
#include <boost/filesystem.hpp>
#include <ros/package.h>

namespace fs = boost::filesystem;

Map::Map(const std::string& file_path, const bl::LocalizationConfig &config) {
  // Set config
   config_ = config;
  //get package path
  std::string package_path = ros::package::getPath("bitbots_localization");
  //make boost path
  fs::path map_path = fs::path(file_path);
  //convert to absolute path
  fs::path absolute_map_path = fs::absolute(map_path, package_path);
  //load map
  map = cv::imread(absolute_map_path.string(), cv::IMREAD_GRAYSCALE);

  if (!map.data) {
    printf("No image data '%s'\n", map_path.filename().c_str());
  }
}

double Map::get_occupancy(double x, double y) {
  //get dimensions of field
  int mapWidth = map.cols;
  int mapHeight = map.rows;

  // m to pixel (=cm)
  x = x * 100;
  y = y * 100;

  // ursprung in feldmitte
  x = std::round(x + mapWidth / 2.0); //assuming lines are centered on map
  y = std::round(y + mapHeight / 2.0);

  double occupancy = -config_.measurement_out_of_map_punishment; // punish points outside the map

  if (x < mapWidth && x >= 0 && y < mapHeight && y >= 0) {
    occupancy = 100 - map.at<uchar>(y, x);
  }
  return occupancy;
}

std::vector<double> Map::provideRating(const RobotState &state,
                                       const std::vector<std::pair<double, double>> &observations) {
  std::vector<double> rating;
  for (const std::pair<double, double> &observation : observations) {
    //lines are in polar form!
    std::pair<double, double> lineRelative;

    // get rating per line
    lineRelative = observationRelative(observation, state.getXPos(), state.getYPos(), state.getTheta());
    double occupancy = get_occupancy(lineRelative.first, lineRelative.second);

    rating.push_back(occupancy);
  }
  return rating;
}

std::pair<double, double> Map::observationRelative(std::pair<double, double> observation,
                                                   double stateX,
                                                   double stateY,
                                                   double stateT) { // todo rename to a more correct name like observationonmap?
  // transformes observation from particle to map (assumes particle is correct)
  // input: obsservation relative in polar coordinates and particle
  // output: hypothetical observation on map

  // add theta and convert back to cartesian
  std::pair<double, double> observationWithTheta = polarToCartesian(observation.first + stateT, observation.second);

  // add to particle
  std::pair<double, double>
      observationRelative = std::make_pair(stateX + observationWithTheta.first, stateY + observationWithTheta.second);

  //alternativ:
  //Thrun 6.32, seite 169
  // but both equivalent
  //double xGlobal = stateX + observation.second * (cos(stateT + observation.first));
  //double yGlobal = stateY + observation.second * (sin(stateT + observation.first));

  //std::pair<double, double> observationRelative = std::make_pair(xGlobal, yGlobal);

  return observationRelative; // in cartesian


}




