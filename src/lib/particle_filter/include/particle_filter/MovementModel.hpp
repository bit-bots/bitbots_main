#ifndef MOVEMENTSTRATEGY_HPP
#define MOVEMENTSTRATEGY_HPP

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/vector3.hpp>

namespace particle_filter {

/**
 * @class MovementModel
 *
 * @brief Templated interface for movement models for particle filters.
 *
 * The movement model in a particle filter defines how a particle's state
 * changes over time. It is used in the drift and diffuse step of
 * particle_filter::ParticleFilter (strategy pattern). To define a movement
 * model, create a sub-class of this class and implement the drift() method. A
 * particle filter with this movement model applies the drift method for each
 * particle in each filter step. Also you have to implement the function
 * diffuse() to define a jitter that is added to a state after drift() (which
 * may be empty of course). You can use the function randomGauss() to obtain
 * Gaussian-distributed random variables.
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
   * @param linear Linear movement of the robot during the last filter step
   * @param angular Angular movement of the robot during the last filter step
   */
  virtual void drift(StateType& state, geometry_msgs::msg::Vector3 linear,
                     geometry_msgs::msg::Vector3 angular) const = 0;

  /**
   * This method will be applied in a ParticleFilter after drift(). It can be
   * used to add a small jitter to the state.
   * @param state Reference to the state that has to be manipulated.
   */
  virtual void diffuse(StateType& state) const = 0;

 private:
};

template <class StateType>
MovementModel<StateType>::~MovementModel() {}

}  // namespace particle_filter
#endif  // MOVEMENTSTRATEGY_HPP
