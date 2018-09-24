#ifndef STATEDISTRIBUTION_H
#define STATEDISTRIBUTION_H
namespace libPF
{

/**
 * @class StateDistribution
 *
 * @brief Templated base class for state distributions.
 *
 * To initialize or reset a particle filter, you may need to use a state
 * distribution to draw your states from. For example, if you do tracking of a
 * position (x, y) in an image a StateDistribution may be used to draw random
 * positions of valid pixels from {(x, y) | x in {0;width-1}, y in {0, height-1}}
 *
 * Use this class as base class for your state distribution and you can use
 * the method ParticleFilter::drawAllFromDistribution().
 *
 * @author Stephan Wirth
 * @see ParticleFilter
 */

template <class StateType>
class StateDistribution {

  public:
    /**
     * The constructor of this base class is empty.
     */
    StateDistribution<StateType>();

    /**
     * The destructor is empty.
     */
    virtual ~StateDistribution();

    /**
     * This is the main method of StateDistribution. It has to return a (random)
     * state, drawn from your distribution. ParticleFilter will copy the state
     * using the assignment operator, so be sure your state supports it!
     * Define this method in your sub-class!
     * @return Drawn state from the distribution.
     */
    virtual const StateType draw() const = 0;

  private:

};


template <class StateType>
StateDistribution<StateType>::StateDistribution() {
}

template <class StateType>
StateDistribution<StateType>::~StateDistribution() {
}

} // end of namespace

#endif // STATEDISTRIBUTION_H
