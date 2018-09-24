#ifndef RANDOMNUMBERGENERATOR_H
#define RANDOMNUMBERGENERATOR_H

#include "libPF/RandomNumberGenerationStrategy.h"

namespace libPF
{

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
class CRandomNumberGenerator : public RandomNumberGenerationStrategy {

  public:

    /**
     * The constructor calls init().
     */
    CRandomNumberGenerator();

    /**
     * Empty destructor.
     */
    ~CRandomNumberGenerator();

    /**
     * This method creates gaussian distributed random numbers (Box-Müller method).
     * @param standardDeviation Standard deviation d of the random number to generate.
     * @return N(0, d*d)-distributed random number
     */
    double getGaussian(double standardDeviation) const;

    /**
     * Generates a uniform distributed random number between min and max.
     * @param min the minimum value, default is 0.0
     * @param max the maximum value, default is 1.0
     * @return random number between min and max, uniform distributed.
     */
    double getUniform(double min = 0.0, double max = 1.0) const;

  protected:

    /**
     * Initializes the seed by calling srand(time(0))
     */
    void init();

  private:

    /// stores if there is a buffered gaussian variable or not
    mutable bool m_GaussianBufferFilled;

    /// buffer for a gaussian distributed variable, the Box-Müller method always
    /// creates two variables, we return one and store the other here.
    mutable double m_GaussianBufferVariable;

};

} // end of namespace

#endif // RANDOMNUMBERGENERATOR_H
