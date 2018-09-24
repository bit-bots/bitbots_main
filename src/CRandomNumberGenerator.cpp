#include <cstdlib>
#include <cmath>
#include <ctime>


#include "libPF/CRandomNumberGenerator.h"

using namespace libPF;

/**
 * @class CRandomNumberGenerator
 *
 * @brief Class for the generation of random numbers.
 *
 * This class can generate randomly generated numbers from uniform and
 * gaussian distributions.
 * Note: this is a very simple PRNG, using the C-function rand().
 *
 * @author Stephan Wirth
 */

CRandomNumberGenerator::CRandomNumberGenerator()
{
    init();
}

CRandomNumberGenerator::~CRandomNumberGenerator()
{
}

void CRandomNumberGenerator::init()
{
    srand(time(0));
}

double CRandomNumberGenerator::getGaussian(double standardDeviation) const
{

  if (standardDeviation < 0) {
    standardDeviation = -standardDeviation;
  }

  if (m_GaussianBufferFilled == true)
  {
    m_GaussianBufferFilled = false;
    return standardDeviation * m_GaussianBufferVariable;
  }
  double x1, x2, w, y1, y2;
  do {
    x1 = getUniform(-1.0, 1.0);
    x2 = getUniform(-1.0, 1.0);
    w = x1 * x1 + x2 * x2;
  } while ( w >= 1.0 );

  w = sqrt((-2.0 * log(w)) / w);
  y1 = x1 * w;
  y2 = x2 * w;
  // now y1 and y2 are N(0,1) distributed
  // we use only one, so we store the other
  m_GaussianBufferVariable = y2;
  m_GaussianBufferFilled = true;
  return standardDeviation * y1;
}

double CRandomNumberGenerator::getUniform(double min, double max) const
{
    double range = max - min;
    return 1.0 * rand() / RAND_MAX * range + min;
}
