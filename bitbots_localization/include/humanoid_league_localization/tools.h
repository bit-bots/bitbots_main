#ifndef HUMANOID_LEAGUE_LOCALIZATION_LOCALIZATION_H
#define HUMANOID_LEAGUE_LOCALIZATION_LOCALIZATION_H


#include <vector>
#include <memory>

#include <ros/ros.h>
#include <math.h>

std::pair<double, double> cartesianToPolar(double x, double y);
std::pair<double, double> polarToCartesian(double t, double r);


#endif //HUMANOID_LEAGUE_LOCALIZATION_LOCALIZATION_H
