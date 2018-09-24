#ifndef IMPORTANCERESAMPLING_H
#define IMPORTANCERESAMPLING_H

#include "libPF/CRandomNumberGenerator.h"

namespace libPF
{

/** 
 * @class ImportanceResampling
 *
 * @brief A resampling strategy that performs importance resampling
 *
 * The resampling strategy defines how the resampling is performed in the resample step
 * of a particle filter.
 * 
 * @author Stephan Wirth
 *
 * @see ResamplingStrategy
 */

template <class StateType>
class ImportanceResampling : public ResamplingStrategy<StateType>{

    /**
     * A ParticleList is an array of pointers to Particles.
     */
    typedef std::vector< Particle<StateType>* > ParticleList;
    
  public:
    /** 
     * The constructor of this base class inits some members.
     */
    ImportanceResampling<StateType>();

    /**
     * The destructor is empty.
     */
    virtual ~ImportanceResampling();

    /**
     * This is the main method of ImportanceResampling. It takes two references to
     * particle lists. The first reference refers to the old particle list, the
     * second to the new one.
     * @param source the source list to draw new particles from.
     * @param destination the destination list where to put the copies.
     */
    void resample(const ParticleList& source, const ParticleList& destination) const;

    /**
     * Sets the Random Number Generator to use in resample() to generate uniformly distributed random numbers.
     */
    void setRNG(RandomNumberGenerationStrategy* rng);

  private:

    // Stores a pointer to the random number generator.
    const RandomNumberGenerationStrategy* m_RNG;

    // The default random number generator
    CRandomNumberGenerator m_DefaultRNG;

};


template <class StateType>
ImportanceResampling<StateType>::ImportanceResampling() :
    m_RNG(&m_DefaultRNG) {
}

template <class StateType>
ImportanceResampling<StateType>::~ImportanceResampling() {
}


// resampling based on the cumulative distribution function (CDF)
template <class StateType>
void ImportanceResampling<StateType>::resample(const ParticleList& sourceList, const ParticleList& destinationList) const {

  double inverseNum = 1.0f / sourceList.size();
  double start = m_RNG->getUniform() * inverseNum;  // random start in CDF
  double cumulativeWeight = 0.0f;
  unsigned int sourceIndex = 0;                     // index to draw from
  cumulativeWeight += sourceList[sourceIndex]->getWeight();
  for (unsigned int destIndex = 0; destIndex < destinationList.size(); destIndex++) {
    double probSum = start + inverseNum * destIndex;     // amount of cumulative weight to reach
    while (probSum > cumulativeWeight) {                 // sum weights until
      sourceIndex++;
      if (sourceIndex >= sourceList.size()) {
        sourceIndex = sourceList.size() - 1;
        break;
      }
      cumulativeWeight += sourceList[sourceIndex]->getWeight(); // target sum reached
    }
    *(destinationList[destIndex]) = *(sourceList[sourceIndex]);  // copy particle (via assignment operator)
  }
}


template <class StateType>
void ImportanceResampling<StateType>::setRNG(RandomNumberGenerationStrategy* rng)
{
    m_RNG = rng;
}

} // end of namespace
#endif // IMPORTANCERESAMPLING_H

