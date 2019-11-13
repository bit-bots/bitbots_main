/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SPLINE_HPP_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SPLINE_HPP_

#include <vector>
#include <iostream>
#include "Polynom.hpp"

namespace bitbots_splines {

/**
 * Spline
 *
 * Generic one dimentional 
 * polynomial spline generator
 */
class Spline {
 public:

  /**
   * Internal spline part structure
   * with a polynom valid on an interval
   */
  struct SplineT {
    Polynom polynom;
    double min;
    double max;
  };

  /**
   * Return spline interpolation
   * at given t. Compute spline value,
   * its first, second and third derivative
   */
  double pos(double t) const;
  double vel(double t) const;
  double acc(double t) const;
  double jerk(double t) const;

  /**
   * Return spline interpolation
   * value, first, second and third derivative
   * with given t bound between 0 and 1
   */
  double posMod(double t) const;
  double velMod(double t) const;
  double accMod(double t) const;
  double jerkMod(double t) const;

  /**
   * Return minimum and maximum abscisse
   * value for which spline is defined
   */
  double min() const;
  double max() const;

  /**
   * Write and read splines data into given
   * iostream in ascii format
   */
  void exportData(std::ostream &os) const;
  void importData(std::istream &is);

  /**
   * Return the number of internal polynom
   */
  size_t size() const;

  /**
   * Access to given by its index
   */
  const SplineT &part(size_t index) const;

  /**
   * Add a part with given polynom
   * and min/max time range
   */
  void addPart(const Polynom &poly,
               double min, double max);

  /**
   * Replace this spline part with the
   * internal data of given spline
   */
  void copyData(const Spline &sp);

 protected:

  /**
   * Spline part container
   */
  std::vector<SplineT> splines_;

  /**
   * Possible override callback
   * after importation
   */
  virtual void importCallBack();

 private:

  /**
   * Return spline interpolation of given value and
   * used given polynom evaluation function
   * (member function pointer)
   */
  double interpolation(double x,
                       double(Polynom::*func)(double) const) const;

  /**
   * Return interpolation with x
   * bound between 0 and 1
   */
  double interpolationMod(double x,
                          double(Polynom::*func)(double) const) const;
};

}

#endif

