#include <bitbots_localization/tools.hpp>

namespace bitbots_localization {

void cartesianToPolar(double x, double y, double& angle, double& radius) {
  radius = hypot(x, y);
  angle = atan2(y, x);
}

void polarToCartesian(double t, double r, double& x, double& y) {
  x = r * std::cos(t);
  y = r * std::sin(t);
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
