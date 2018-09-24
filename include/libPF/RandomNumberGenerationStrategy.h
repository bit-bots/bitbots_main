#ifndef RANDOMNUMBERGENERATIONSTRATEGY_H
#define RANDOMNUMBERGENERATIONSTRATEGY_H

namespace libPF
{

/**
 * @class RandomNumberGenerationStrategy
 *
 * @brief Interface for a strategy to create random numbers
 *
 * This class defines the interface for random number generators. To create your
 * own random number generation strategy, sub-class from this class.
 *
 * @author Stephan Wirth
 */
class RandomNumberGenerationStrategy {

  public:

    /**
     * Empty constructor.
     */
    RandomNumberGenerationStrategy() {};

    /**
     * Empty destructor.
     */
    virtual ~RandomNumberGenerationStrategy() {};

    /**
     * Interface for the generation function of gaussian distributed numbers.
     * @param standardDeviation Standard deviation d of the random number to generate.
     * @return N(0, d*d)-distributed random number
     */
    virtual double getGaussian(double standardDeviation) const = 0;

    /**
     * Interface for the generation of a uniform distributed random number between min and max.
     * @param min the minimum value, default is 0.0
     * @param max the maximum value, default is 1.0
     * @return random number between min and max, uniform distributed.
     */
    virtual double getUniform(double min = 0.0, double max = 1.0) const = 0;

  protected:

  private:

};

} // end of namespace

#endif // RANDOMNUMBERGENERATIONSTRATEGY_H
