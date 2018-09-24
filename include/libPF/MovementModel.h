#ifndef MOVEMENTSTRATEGY_H
#define MOVEMENTSTRATEGY_H

#include <cmath>
#include <algorithm>

namespace libPF
{

/** 
 * @class MovementModel
 *
 * @brief Templated interface for movement models for particle filters.
 *
 * The movement model in a particle filter defines how a particle's state changes
 * over time.
 * It is used in the drift and diffuse step of libPF::ParticleFilter (strategy pattern).
 * To define a movement model, create a sub-class of this class and
 * implement the drift() method. A particle filter with this movement model
 * applies the drift method for each particle in each filter step. Also you have
 * to implement the function diffuse() to define a jitter that is added to
 * a state after drift() (which may be empty of course). You can use the function
 * randomGauss() to obtain Gaussian-distributed random variables.
 * 
 * @author Stephan Wirth
 *
 * @see ParticleFilter
 * @see Particle
 */

template <class StateType>
class MovementModel {
    
  public:

    /**
     * The destructor is empty.
     */
    virtual ~MovementModel();

    /**
     * This is the main method of MovementModel. It takes a state reference as
     * argument and is supposed to extract the state's variables and manipulate
     * them. dt means delta t and defines the time in seconds that has passed
     * since the last filter update.
     * Define this function in your sub-class!
     * @param state Reference to the state that has to be manipulated.
     * @param dt time that has passed since the last filter update in seconds.
     */
    virtual void drift(StateType& state, double dt) const = 0;

    /**
     * This method will be applied in a ParticleFilter after drift(). It can be
     * used to add a small jitter to the state.
     * @param state Reference to the state that has to be manipulated.
     * @param dt time that has passed since the last filter update in seconds.
     */
    virtual void diffuse(StateType& state, double dt) const = 0;

  private:

};

template <class StateType>
MovementModel<StateType>::~MovementModel() {
}

} // end of namespace
#endif

