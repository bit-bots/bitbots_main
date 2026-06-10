#include <bitbots_localization/tools.hpp>
#include <cmath>

namespace bitbots_localization {

PolarCoordinates cartesianToPolar(const CartesianCoordinates& coordinates) {
  return {atan2(coordinates.y, coordinates.x), hypot(coordinates.x, coordinates.y)};
}

PolarCoordinates cartesianToPolar(double x, double y) { return cartesianToPolar({x, y}); }

CartesianCoordinates polarToCartesian(const PolarCoordinates& coordinates) {
  return {coordinates.radius * std::cos(coordinates.angle), coordinates.radius * std::sin(coordinates.angle)};
}

CartesianCoordinates polarToCartesian(double angle, double radius) { return polarToCartesian({angle, radius}); }

double signedAngle(double angle) {
  if (angle > M_PI) {
    angle -= M_PI * 2;
  }
  if (angle < -M_PI) {
    angle += M_PI * 2;
  }
  return angle;
}

double signedAngle(double angle_a, double angle_b) { return signedAngle(angle_a - angle_b); }
}  // namespace bitbots_localization
