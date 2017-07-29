#include <iomanip>
#include <stdexcept>
#include "Spline.hpp"

namespace Leph {

double Spline::pos(double t) const
{
    return interpolation(t, &Polynom::pos);
}
double Spline::vel(double t) const
{
    return interpolation(t, &Polynom::vel);
}
double Spline::acc(double t) const
{
    return interpolation(t, &Polynom::acc);
}
        
double Spline::posMod(double t) const
{
    return interpolationMod(t, &Polynom::pos);
}
double Spline::velMod(double t) const
{
    return interpolationMod(t, &Polynom::vel);
}
double Spline::accMod(double t) const
{
    return interpolationMod(t, &Polynom::acc);
}
        
double Spline::min() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.front().min;
    }
}
double Spline::max() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.back().max;
    }
}

void Spline::exportData(std::ostream& os) const
{
    for (size_t i=0;i<_splines.size();i++) {
        os << std::setprecision(10) << _splines[i].min << " ";
        os << std::setprecision(10) << _splines[i].max << " ";
        os << std::setprecision(10) << 
            _splines[i].polynom.getCoefs().size() << " ";
        for (size_t j=0;j<_splines[i].polynom.getCoefs().size();j++) {
            os << std::setprecision(10) << 
                _splines[i].polynom.getCoefs()[j] << " ";
        }
    }
    os << std::endl;
}
void Spline::importData(std::istream& is)
{
    bool isFormatError;
    while (is.good()) {
        isFormatError = true;
        double min;
        double max;
        size_t size;
        Polynom p;
        //Load spline interval and degree
        is >> min;
        if (!is.good()) break;
        is >> max;
        if (!is.good()) break;
        is >> size;
        //Load polynom coeficients
        p.getCoefs().resize(size);
        for (size_t i=0;i<size;i++) {
            if (!is.good()) break;
            is >> p.getCoefs()[i];
        }
        //Save spline part
        isFormatError = false;
        _splines.push_back({p, min, max});
        //Exit on line break
        while (is.peek() == ' ') {
            if (!is.good()) break;
            is.ignore();
        }
        if (is.peek() == '\n') {
            break;
        }
    }
    if (isFormatError) {
        throw std::logic_error(
            "Spline import format invalid");
    }
}

double Spline::interpolation(double x, 
    double(Polynom::*func)(double) const) const
{
    //Bound asked abscisse into spline range
    if (x <= _splines.front().min) {
        x = _splines.front().min;
    }
    if (x >= _splines.back().max) {
        x = _splines.back().max;
    }
    //Bijection spline search
    size_t indexLow = 0;
    size_t indexUp = _splines.size()-1;
    while (indexLow != indexUp) {
        size_t index = (indexUp+indexLow)/2;
        if (x < _splines[index].min) {
            indexUp = index-1;
        } else if (x > _splines[index].max) {
            indexLow = index+1;
        } else {
            indexUp = index;
            indexLow = index;
        }
    }
    //Compute and return spline value
    return (_splines[indexUp].polynom.*func)
        (x-_splines[indexUp].min);
}

double Spline::interpolationMod(double x, 
    double(Polynom::*func)(double) const) const
{
    if (x < 0.0) {
        x = 1.0 + (x - ((int)x/1));
    } else if (x > 1.0) {
        x = (x - ((int)x/1));
    }
    return interpolation(x, func);
}

}

