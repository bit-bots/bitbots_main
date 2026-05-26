#ifndef BITBOTS_LOCALIZATION_TOOLS_H
#define BITBOTS_LOCALIZATION_TOOLS_H

#include <math.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bitbots_localization {
void cartesianToPolar(double x, double y, double& angle, double& radius);
void polarToCartesian(double t, double r, double& x, double& y);
double signedAngle(double angle_a, double angle_b);
double signedAngle(double angle);
}  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_TOOLS_H
