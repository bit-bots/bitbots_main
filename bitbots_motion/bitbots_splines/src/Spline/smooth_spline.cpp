/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_splines/smooth_spline.hpp"

#include <math.h>

#include <algorithm>
#include <stdexcept>

namespace bitbots_splines {

void SmoothSpline::addPoint(double time, double position, double velocity) {
  points_.push_back({time, position, velocity});
  computeSplines();
}

const std::vector<SmoothSpline::Point> &SmoothSpline::points() const { return points_; }
std::vector<SmoothSpline::Point> &SmoothSpline::points() { return points_; }

void SmoothSpline::computeSplines() {
  Spline::splines_.clear();
  if (points_.size() < 2) {
    return;
  }

  std::sort(points_.begin(), points_.end(),
            [](const Point &p_1, const Point &p_2) -> bool { return p_1.time < p_2.time; });

  for (size_t i = 1; i < points_.size(); i++) {
    double time = points_[i].time - points_[i - 1].time;
    if (time > 0.00001) {
      Spline::splines_.push_back(
          {polynomFit(time, points_[i - 1].position, points_[i - 1].velocity, points_[i].position, points_[i].velocity),
           points_[i - 1].time, points_[i].time});
    }
  }
}

void SmoothSpline::importCallBack() {
  size_t size = Spline::splines_.size();
  if (size == 0) {
    return;
  }

  double t_begin = Spline::splines_.front().min;
  points_.push_back({t_begin, Spline::pos(t_begin), Spline::vel(t_begin)});

  for (size_t i = 1; i < size; i++) {
    double t_1 = Spline::splines_[i - 1].max;
    double t_2 = Spline::splines_[i].min;
    double pos_1 = Spline::pos(t_1);
    double vel_1 = Spline::vel(t_1);
    double pos_2 = Spline::pos(t_2);
    double vel_2 = Spline::vel(t_2);

    if (fabs(t_2 - t_1) < 0.0001 && fabs(pos_2 - pos_1) < 0.0001 && fabs(vel_2 - vel_1) < 0.0001) {
      points_.push_back({t_1, pos_1, vel_1});
    } else {
      points_.push_back({t_1, pos_1, vel_1});
      points_.push_back({t_2, pos_2, vel_2});
    }
  }

  double t_end = Spline::splines_.back().max;
  points_.push_back({t_end, Spline::pos(t_end), Spline::vel(t_end)});
}

Polynom SmoothSpline::polynomFit(double t, double pos_1, double vel_1, double pos_2, double vel_2) const {
  if (t <= 0.00001) {
    throw std::logic_error("SmoothSpline invalid spline interval");
  }
  double t_2 = t * t;
  double t_3 = t_2 * t;

  Polynom p;
  p.getCoefs().resize(4);
  p.getCoefs()[0] = pos_1;
  p.getCoefs()[1] = vel_1;
  p.getCoefs()[2] = (3 * (pos_2 - pos_1) / (t_2) - (2 * vel_1 + vel_2) / t);
  p.getCoefs()[3] = (2 * (pos_1 - pos_2) / (t_3) + (vel_1 + vel_2) / (t_2));

  return p;
}

std::string SmoothSpline::getDebugString() {
  std::string output;
  int i = 0;
  for (auto &p : points_) {
    output += "Point:" + std::to_string(i) + "\n";
    output += "  Time: " + std::to_string(p.time) + "\n";
    output += "  Pos: " + std::to_string(p.position) + "\n";
    output += "  Vel: " + std::to_string(p.velocity) + "\n";
    i++;
  }
  return output;
}

}  // namespace bitbots_splines
