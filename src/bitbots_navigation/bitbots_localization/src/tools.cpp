#include <bitbots_localization/tools.hpp>

namespace bitbots_localization {

PolarCoordinates cartesianToPolar(double x, double y) { return {atan2(y, x), hypot(x, y)}; }

CartesianCoordinates polarToCartesian(double angle, double radius) {
  return {radius * std::cos(angle), radius * std::sin(angle)};
}

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
