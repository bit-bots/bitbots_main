#ifndef OBSERVATIONSTRATEGY_HPP
#define OBSERVATIONSTRATEGY_HPP

#include <cmath>

namespace particle_filter {

/**
 * @class ObservationModel
 *
 * @brief Templated interface for observation models for particle filters.
 *
 * The observation model in a particle filter defines the measurement process
 * that is used to weighten the particles according to their state.
 * It is used in the measurement step of the particle filter
 * particle_filter::ParticleFilter::measure() (strategy pattern). To define an
 * observation model, create a sub-class of this class, specializing the
 * measurement method and the state type to use (with specializing the
 * template). The measurement method takes a reference to a state as an
 * argument. Use this reference to extract the state's variables and use your
 * measurement function to compute a state-dependent weight. The weight has to
 * be a positive, non-zero value.
 *
 * @author Stephan Wirth
 * @see ParticleFilter
 * @see Particle
 */

template <class StateType>
class ObservationModel {
 public:
  /**
   * The destructor is empty.
   */
  virtual ~ObservationModel();

  /**
   * This is the main method of ObservationModel. It takes a state reference
   * as argument and is supposed to extract the state's variables to compute
   * an importance weight for the state.
   * Define this method in your sub-class!
   * @param state Reference to the state that has to be weightened.
   * @return importance weight for the given state (positive, non-zero value).
   */
  virtual double measure(const StateType& state) const = 0;

  virtual bool measurements_available() = 0;
  // virtual void clear_measurement();
  virtual double get_min_weight() const = 0;

  bool accumulate_weights_ = false;

 private:
};

template <class StateType>
ObservationModel<StateType>::~ObservationModel() {}

}  // namespace particle_filter
#endif  // OBSERVATIONSTRATEGY_HPP
