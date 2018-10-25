#ifndef IMPORTANCERESAMPLINGWE_H
#define IMPORTANCERESAMPLINGWE_H

#include <assert.h>
#include "libPF/ImportanceResampling.h"
#include "libPF/CRandomNumberGenerator.h"

// ImportanceResampling with explorers

template <class StateType>
class ImportanceResamplingWE : public libPF::ImportanceResampling <StateType> {

    /**
     * A ParticleList is an array of pointers to Particles.
     */
    typedef std::vector< libPF::Particle<StateType>* > ParticleList;

  public:
    /**
     * The constructor of this base class inits some members.
     */
    ImportanceResamplingWE<StateType>(int explorer_count, std::shared_ptr<libPF::StateDistribution<StateType>> distribution);

    /**
     * The destructor is empty.
     */
    virtual ~ImportanceResamplingWE();

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
    void setExplorerCount(int explorer_count);

    int getExplorerCount();

  private:

    int explorer_count_;
    std::shared_ptr<libPF::StateDistribution<StateType>> distribution_;

    // Stores a pointer to the random number generator.
    const libPF::RandomNumberGenerationStrategy* m_RNG;

    // The default random number generator
    libPF::CRandomNumberGenerator m_DefaultRNG;

};


template <class StateType>
ImportanceResamplingWE<StateType>::ImportanceResamplingWE(int explorer_count, std::shared_ptr<libPF::StateDistribution<StateType>> distribution) :
    explorer_count_(explorer_count),
    distribution_(distribution) {
    libPF::ImportanceResampling<StateType>();
}

template <class StateType>
ImportanceResamplingWE<StateType>::~ImportanceResamplingWE() {
}


// resampling based on the cumulative distribution function (CDF)
// this is an implementation of the algorithm presented in Propabilistic Robotics by Sebastian Thrun et al.
template <class StateType>
void ImportanceResamplingWE<StateType>::resample(const ParticleList& sourceList, const ParticleList& destinationList) const {
  assert(sourceList.size() == destinationList.size());
  // some particles (most of them usually) get resampled and explorer_count particles get assigned a random state
  int resample_max = sourceList.size() - explorer_count_;
  double inverseNum = 1.0f / resample_max;
  double start = m_RNG->getUniform() * inverseNum;  // random start in CDF
  double cumulativeWeight = 0.0f;
  unsigned int sourceIndex = 0;                     // index to draw from
  cumulativeWeight += sourceList[sourceIndex]->getWeight();
  for (unsigned int destIndex = 0; destIndex < resample_max; destIndex++) {
    double probSum = start + inverseNum * destIndex;     // amount of cumulative weight to reach
    while (probSum > cumulativeWeight) {                 // sum weights until
      sourceIndex++;
      if (sourceIndex >= resample_max) {
        sourceIndex = resample_max - 1;
        break;
      }
      cumulativeWeight += sourceList[sourceIndex]->getWeight(); // target sum reached
    }
    *(destinationList[destIndex]) = *(sourceList[sourceIndex]);  // copy particle (via assignment operator)
  }

}

//template <class StateType>
//void ParticleFilter<StateType>::drawAllFromDistribution(const std::shared_ptr<StateDistribution<StateType>>& distribution) {

#endif // IMPORTANCERESAMPLINGWE_H

