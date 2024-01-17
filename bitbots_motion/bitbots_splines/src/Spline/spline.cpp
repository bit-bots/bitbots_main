/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <iomanip>
#include <stdexcept>
#include "bitbots_splines/spline.hpp"

namespace bitbots_splines {

double Spline::pos(double t) const {
  return interpolation(t, &Polynom::pos);
}
double Spline::vel(double t) const {
  return interpolation(t, &Polynom::vel);
}
double Spline::acc(double t) const {
  return interpolation(t, &Polynom::acc);
}
double Spline::jerk(double t) const {
  return interpolation(t, &Polynom::jerk);
}

double Spline::posMod(double t) const {
  return interpolationMod(t, &Polynom::pos);
}
double Spline::velMod(double t) const {
  return interpolationMod(t, &Polynom::vel);
}
double Spline::accMod(double t) const {
  return interpolationMod(t, &Polynom::acc);
}
double Spline::jerkMod(double t) const {
  return interpolationMod(t, &Polynom::jerk);
}

double Spline::min() const {
  if (splines_.empty()) {
    return 0.0;
  } else {
    return splines_.front().min;
  }
}
double Spline::max() const {
  if (splines_.empty()) {
    return 0.0;
  } else {
    return splines_.back().max;
  }
}

void Spline::exportData(std::ostream &os) const {
  for (const auto &spline : splines_) {
    os << std::setprecision(17) << spline.min << " ";
    os << std::setprecision(17) << spline.max << " ";
    os << std::setprecision(17) <<
       spline.polynom.getCoefs().size() << " ";
    for (size_t j = 0; j < spline.polynom.getCoefs().size(); j++) {
      os << std::setprecision(17) <<
         spline.polynom.getCoefs()[j] << " ";
    }
  }
  os << std::endl;
}
void Spline::importData(std::istream &is) {
  bool is_format_error;
  while (is.good()) {
    is_format_error = true;
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
    for (size_t i = 0; i < size; i++) {
      if (!is.good()) break;
      is >> p.getCoefs()[i];
    }
    //Save spline part
    is_format_error = false;
    splines_.push_back({p, min, max});
    //Exit on line break
    while (is.peek() == ' ') {
      if (!is.good()) break;
      is.ignore();
    }
    if (is.peek() == '\n') {
      break;
    }
  }
  if (is_format_error) {
    throw std::logic_error(
        "Spline import format invalid");
  }
  //Call possible post import
  importCallBack();
}

size_t Spline::size() const {
  return splines_.size();
}

const Spline::SplineT &Spline::part(size_t index) const {
  return splines_.at(index);
}

void Spline::addPart(const Polynom &poly,
                     double min, double max) {
  splines_.push_back({poly, min, max});
}

void Spline::copyData(const Spline &sp) {
  splines_ = sp.splines_;
  //Call possible post import
  importCallBack();
}

void Spline::importCallBack() {
}

double Spline::interpolation(double x,
                             double(Polynom::*func)(double) const) const {
  //Empty case
  if (splines_.empty()) {
    return 0.0;
  }
  //Bound asked abscisse into spline range
  if (x <= splines_.front().min) {
    x = splines_.front().min;
  }
  if (x >= splines_.back().max) {
    x = splines_.back().max;
  }
  //Bijection spline search
  size_t index_low = 0;
  size_t index_up = splines_.size() - 1;
  while (index_low != index_up) {
    size_t index = (index_up + index_low) / 2;
    if (x < splines_[index].min) {
      index_up = index - 1;
    } else if (x > splines_[index].max) {
      index_low = index + 1;
    } else {
      index_up = index;
      index_low = index;
    }
  }
  //Compute and return spline value
  return (splines_[index_up].polynom.*func)
      (x - splines_[index_up].min);
}

double Spline::interpolationMod(double x,
                                double(Polynom::*func)(double) const) const {
  if (x < 0.0) {
    x = 1.0 + (x - ((int) x / 1));
  } else if (x > 1.0) {
    x = (x - ((int) x / 1));
  }
  return interpolation(x, func);
}

}

