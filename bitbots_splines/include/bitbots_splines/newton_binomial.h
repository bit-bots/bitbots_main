/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_NEWTON_BINOMIAL_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_NEWTON_BINOMIAL_H_

#include "combination.h"
#include "polynom.h"

namespace bitbots_splines {

/**
 * NewtonBinomial
 *
 * Implement Newton binomial
 * simple formulae and binding
 * with polynom structure
 */
class NewtonBinomial {
 public:

  /**
   * Expand the given formula (x + y)^degree
   * and return the polynom in x whose coefficient
   * are computed using binomial coefficient
   */
  static Polynom expandPolynom(
      double y, unsigned int degree);

 private:
};

}

#endif

