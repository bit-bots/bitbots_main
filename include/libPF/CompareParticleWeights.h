#ifndef COMPAREPARTICLEWEIGHTS_H
#define COMPAREPARTICLEWEIGHTS_H  

#include "libPF/Particle.h"

namespace libPF
{

/** 
 * @class CompareParticleWeights
 *
 * @author Stephan Wirth
 *
 * @brief Class with one operator to compare two pointers of particles according
 *        to the weight of the particles.
 *
 * With this class as compare function, std::sort() can be used on arrays of
 * pointers to Particle. After sorting the array with this function, the particle
 * with the smallest weight will be at the last position.
 * 
 * @see Particle
 */
template <class StateType>
class CompareParticleWeights {

  public:

    /**
     * @return true if the weight of the particle p1 is higher than the weight of particle p2.
     */
    bool operator() (const libPF::Particle<StateType>* p1,
                     const libPF::Particle<StateType>* p2) const
    {
      return p1->getWeight() > p2->getWeight();
    }
};

} // end of namespace

#endif


