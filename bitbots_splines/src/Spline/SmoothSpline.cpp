/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include "bitbots_splines/SmoothSpline.hpp"


namespace bitbots_splines {

void SmoothSpline::addPoint(double time, double position, 
    double velocity, double acceleration)
{
    _points.push_back({time, position, 
        velocity, acceleration});
    computeSplines();
}

const std::vector<SmoothSpline::Point>& SmoothSpline::points() const
{
    return _points;
}
std::vector<SmoothSpline::Point>& SmoothSpline::points()
{
    return _points;
}

void SmoothSpline::computeSplines() 
{
    Spline::_splines.clear();
    if (_points.size() < 2) {
        return;
    }

    std::sort(
        _points.begin(), 
        _points.end(), 
        [](const Point& p1, const Point& p2) -> bool { 
            return p1.time < p2.time;
        });

    for (size_t i=1;i<_points.size();i++) {
        double time = _points[i].time - _points[i-1].time;
        if (time > 0.00001) {
            Spline::_splines.push_back({
                polynomFit(time,
                    _points[i-1].position, _points[i-1].velocity, _points[i-1].acceleration,
                    _points[i].position, _points[i].velocity, _points[i].acceleration),
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

void SmoothSpline::importCallBack()
{
    size_t size = Spline::_splines.size();
    if (size == 0) {
        return;
    }

    double tBegin = Spline::_splines.front().min;
    _points.push_back({
        tBegin, 
        Spline::pos(tBegin), 
        Spline::vel(tBegin),
        Spline::acc(tBegin)
    });

    for (size_t i=1;i<size;i++) {
        double t1 = Spline::_splines[i-1].max;
        double t2 = Spline::_splines[i].min;
        double pos1 = Spline::pos(t1);
        double vel1 = Spline::vel(t1);
        double acc1 = Spline::acc(t1);
        double pos2 = Spline::pos(t2);
        double vel2 = Spline::vel(t2);
        double acc2 = Spline::acc(t2);

        if (
            fabs(t2-t1) < 0.0001 && 
            fabs(pos2-pos1) < 0.0001 &&
            fabs(vel2-vel1) < 0.0001 &&
            fabs(acc2-acc1) < 0.0001
        ) {
            _points.push_back({t1, pos1, vel1, acc1});
        } else {
            _points.push_back({t1, pos1, vel1, acc1});
            _points.push_back({t2, pos2, vel2, acc2});
        }
    }

    double tEnd = Spline::_splines.back().max;
    _points.push_back({
        tEnd, 
        Spline::pos(tEnd), 
        Spline::vel(tEnd),
        Spline::acc(tEnd)
    });
}

Polynom SmoothSpline::polynomFit(double t, 
    double pos1, double vel1, double acc1,
    double pos2, double vel2, double acc2) const
{
    if (t <= 0.00001) {
        throw std::logic_error(
            "SmoothSpline invalid spline interval");
    }
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;
    Polynom p;
    p.getCoefs().resize(6);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[2] = acc1/2;
    p.getCoefs()[3] = -(-acc2*t2+3*acc1*t2+8*vel2*t+12*vel1*t-20*pos2+20*pos1)/(2*t3);
    p.getCoefs()[4] = (-2*acc2*t2+3*acc1*t2+14*vel2*t+16*vel1*t-30*pos2+30*pos1)/(2*t4);
    p.getCoefs()[5] = -(-acc2*t2+acc1*t2+6*vel2*t+6*vel1*t-12*pos2+12*pos1)/(2*t5);

    return p;
}

std::string SmoothSpline::getDebugString(){
  std::string output;
  int i = 0;
  for(auto &p : _points){
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

