//
// Created by judith on 08.03.19.
//

#ifndef HUMANOID_LEAGUE_LOCALIZATION_MAP_H
#define HUMANOID_LEAGUE_LOCALIZATION_MAP_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <humanoid_league_localization/RobotState.h>
#include <geometry_msgs/Point.h>
#include <humanoid_league_localization/tools.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>



namespace gm = geometry_msgs;

class Map {
public:
    Map(std::string file_path);

    cv::Mat map;

    std::vector<double> provideRating(const RobotState &state, const std::vector<std::pair<double, double>> &observations);

    double get_occupancy(double x, double y);

    std::pair<double, double> observationRelative(std::pair<double, double> observation, double stateX, double stateY, double stateT);


private:

};


#endif //HUMANOID_LEAGUE_LOCALIZATION_MAP_H
