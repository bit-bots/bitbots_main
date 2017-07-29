#include <stdexcept>
#include <algorithm>
#include "CubicSpline.hpp"

namespace Leph {

void CubicSpline::addPoint(double time, double position, 
    double velocity)
{
    _points.push_back({time, 
        position, velocity});
    computeSplines();
}

Polynom CubicSpline::polynomFit(double t, 
    double pos1, double vel1,
    double pos2, double vel2) const
{
    if (t <= 0.00001) {
        throw std::logic_error(
            "CubicSpline invalid spline interval");
    }
    double t2 = t*t;
    double t3 = t2*t;
    Polynom p;
    p.getCoefs().resize(4);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[3] = (vel2 - vel1 - 2.0*(pos2-pos1-vel1*t)/t)/t2;
    p.getCoefs()[2] = (pos2 - pos1 - vel1*t - p.getCoefs()[3]*t3)/t2;

    return p;
}

void CubicSpline::computeSplines() 
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
                    _points[i-1].position, _points[i-1].velocity,
                    _points[i].position, _points[i].velocity),
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

}

