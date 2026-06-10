#ifndef BITBOTS_LOCALIZATION_TOOLS_H
#define BITBOTS_LOCALIZATION_TOOLS_H

#include <math.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace bitbots_localization {
struct PolarCoordinates {
  double angle;
  double radius;
};

struct CartesianCoordinates {
  double x;
  double y;
};

PolarCoordinates cartesianToPolar(const CartesianCoordinates& coordinates);
PolarCoordinates cartesianToPolar(double x, double y);
CartesianCoordinates polarToCartesian(const PolarCoordinates& coordinates);
CartesianCoordinates polarToCartesian(double angle, double radius);
double signedAngle(double angle_a, double angle_b);
double signedAngle(double angle);
}  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_TOOLS_H
