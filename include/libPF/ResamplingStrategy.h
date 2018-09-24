#ifndef RESAMPLINGSTRATEGY_H
#define RESAMPLINGSTRATEGY_H

#include <vector>

#include "libPF/Particle.h"

namespace libPF
{

/** 
 * @class ResamplingStrategy
 *
 * @brief Templated interface for resampling strategies
 *
 * The resampling strategy defines how the resampling is performed in the resample step
 * of a particle filter. One implementation of this strategy is @see ImportanceResampling.
 * 
 * @author Stephan Wirth
 *
 * @see ParticleFilter
 */

template <class StateType>
class ResamplingStrategy {

    /**
     * A ParticleList is an array of pointers to Particles.
     */
    typedef std::vector< Particle<StateType>* > ParticleList;
    
  public:

    /**
     * The destructor is empty.
     */
    virtual ~ResamplingStrategy();

    /**
     * This is the main method of ResamplingStrategy. It takes two references to
     * particle lists. The first reference refers to the old particle list, the
     * second to the new one. The strategy has to define which particles have to be copied to the new list.
     * Use the assignment operator to copy a particle. Be careful that you don't copy
     * the pointer to the particle! Never change the size of the lists!
     * Define this function in your sub-class!
     * @param source the source list to draw new particles from.
     * @param destination the destination list where to put the copies.
     */
    virtual void resample(const ParticleList& source, const ParticleList& destination) const = 0;

  private:

};

template <class StateType>
ResamplingStrategy<StateType>::~ResamplingStrategy() {
}

} // end of namespace
#endif // RESAMPLINGSTRATEGY_H

