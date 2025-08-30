#ifndef BITBOTS_LOCALIZATION_TOOLS_H
#define BITBOTS_LOCALIZATION_TOOLS_H

#include <math.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bitbots_localization {
std::pair<double, double> cartesianToPolar(double x, double y);
std::pair<double, double> polarToCartesian(double t, double r);
double signedAngle(double angle_a, double angle_b);
double signedAngle(double angle);
}  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_TOOLS_H
