/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include "bitbots_splines/smooth_spline.h"

namespace bitbots_splines {

void SmoothSpline::addPoint(double time, double position,
                            double velocity, double acceleration) {
  points_.push_back({time, position,
                     velocity, acceleration});
  computeSplines();
}

const std::vector<SmoothSpline::Point> &SmoothSpline::points() const {
  return points_;
}
std::vector<SmoothSpline::Point> &SmoothSpline::points() {
  return points_;
}

void SmoothSpline::computeSplines() {
  Spline::splines_.clear();
  if (points_.size() < 2) {
    return;
  }

  std::sort(
      points_.begin(),
      points_.end(),
      [](const Point &p_1, const Point &p_2) -> bool {
        return p_1.time < p_2.time;
      });

  for (size_t i = 1; i < points_.size(); i++) {
    double time = points_[i].time - points_[i - 1].time;
    if (time > 0.00001) {
      Spline::splines_.push_back({
                                     polynomFit(time,
                                                points_[i - 1].position,
                                                points_[i - 1].velocity,
                                                points_[i - 1].acceleration,
                                                points_[i].position,
                                                points_[i].velocity,
                                                points_[i].acceleration),
                                     points_[i - 1].time,
                                     points_[i].time
                                 });
    }
  }
}

void SmoothSpline::importCallBack() {
  size_t size = Spline::splines_.size();
  if (size == 0) {
    return;
  }

  double t_begin = Spline::splines_.front().min;
  points_.push_back({
                        t_begin,
                        Spline::pos(t_begin),
                        Spline::vel(t_begin),
                        Spline::acc(t_begin)
                    });

  for (size_t i = 1; i < size; i++) {
    double t_1 = Spline::splines_[i - 1].max;
    double t_2 = Spline::splines_[i].min;
    double pos_1 = Spline::pos(t_1);
    double vel_1 = Spline::vel(t_1);
    double acc_1 = Spline::acc(t_1);
    double pos_2 = Spline::pos(t_2);
    double vel_2 = Spline::vel(t_2);
    double acc_2 = Spline::acc(t_2);

    if (
        fabs(t_2 - t_1) < 0.0001 &&
            fabs(pos_2 - pos_1) < 0.0001 &&
            fabs(vel_2 - vel_1) < 0.0001 &&
            fabs(acc_2 - acc_1) < 0.0001
        ) {
      points_.push_back({t_1, pos_1, vel_1, acc_1});
    } else {
      points_.push_back({t_1, pos_1, vel_1, acc_1});
      points_.push_back({t_2, pos_2, vel_2, acc_2});
    }
  }

  double t_end = Spline::splines_.back().max;
  points_.push_back({
                        t_end,
                        Spline::pos(t_end),
                        Spline::vel(t_end),
                        Spline::acc(t_end)
                    });
}

Polynom SmoothSpline::polynomFit(double t,
                                 double pos_1, double vel_1, double acc_1,
                                 double pos_2, double vel_2, double acc_2) const {
  if (t <= 0.00001) {
    throw std::logic_error(
        "SmoothSpline invalid spline interval");
  }
  double t_2 = t * t;
  double t_3 = t_2 * t;
  double t_4 = t_3 * t;
  double t_5 = t_4 * t;
  Polynom p;
  p.getCoefs().resize(6);
  p.getCoefs()[0] = pos_1;
  p.getCoefs()[1] = vel_1;
  p.getCoefs()[2] = acc_1 / 2;
  p.getCoefs()[3] =
      -(-acc_2 * t_2 + 3 * acc_1 * t_2 + 8 * vel_2 * t + 12 * vel_1 * t - 20 * pos_2 + 20 * pos_1) / (2 * t_3);
  p.getCoefs()[4] =
      (-2 * acc_2 * t_2 + 3 * acc_1 * t_2 + 14 * vel_2 * t + 16 * vel_1 * t - 30 * pos_2 + 30 * pos_1) / (2 * t_4);
  p.getCoefs()[5] = -(-acc_2 * t_2 + acc_1 * t_2 + 6 * vel_2 * t + 6 * vel_1 * t - 12 * pos_2 + 12 * pos_1) / (2 * t_5);

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
    output += "  Acc: " + std::to_string(p.acceleration) + "\n";
    i++;
  }
  return output;
}

}

