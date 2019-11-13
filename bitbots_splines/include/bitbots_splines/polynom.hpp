/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POLYNOM_HPP_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POLYNOM_HPP_

#include <cstdlib>
#include <vector>
#include <iostream>

namespace bitbots_splines {

/**
 * Polynom
 *
 * Simple one dimentional 
 * polynom class for spline 
 * generation
 */
class Polynom {
 public:

  /**
   * Default and inital degree initialization
   */
  Polynom();
  explicit Polynom(unsigned int degree);

  /**
   * Access to coefficient
   * indexed from constant to
   * higher degree
   */
  const std::vector<double> &getCoefs() const;
  std::vector<double> &getCoefs();

  /**
   * Access to coefficient
   */
  const double &operator()(size_t index) const;
  double &operator()(size_t index);

  /**
   * Return polynom degree
   * -1 mean empty polynom
   */
  size_t degree() const;

  /**
   * Polynom evaluation, its first,
   * second and third derivative at given x
   */
  double pos(double x) const;
  double vel(double x) const;
  double acc(double x) const;
  double jerk(double x) const;

  /**
   * Some useful operators
   */
  void operator*=(double coef);
  void operator+=(const Polynom &p);

  /**
   * Update the polynom coefficients
   * by applying delta offset
   * on X abscisse
   */
  void shift(double delta);

 private:

  /**
   * Polynom coeficients
   */
  std::vector<double> coefs_;
};

/**
 * Print operator
 */
std::ostream &operator<<(std::ostream &os, const Polynom &p);

}

#endif

