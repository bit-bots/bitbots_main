#include <bitbots_localization/tools.hpp>

namespace bitbots_localization {

std::pair<double, double> cartesianToPolar(double x, double y) {
  double r = hypot(x, y);

  double t = atan2(y, x);

  return std::make_pair(t, r);
}

std::pair<double, double> polarToCartesian(double t, double r) {
  double x = r * std::cos(t);
  double y = r * std::sin(t);

  return std::make_pair(x, y);
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
