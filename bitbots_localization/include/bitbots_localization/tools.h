#ifndef BITBOTS_LOCALIZATION_TOOLS_H
#define BITBOTS_LOCALIZATION_TOOLS_H


#include <vector>
#include <memory>

#include <ros/ros.h>
#include <math.h>

std::pair<double, double> cartesianToPolar(double x, double y);
std::pair<double, double> polarToCartesian(double t, double r);


#endif //BITBOTS_LOCALIZATION_TOOLS_H
