#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <particle_filter/ParticleFilter.hpp>

namespace particle_filter {
/**
 * @class Particle
 * @brief Class that represents a particle for a particle filter.
 *
 * A particle as it is used in particle filters is a set of one state
 * (m_State) and one importance factor (m_Weight).
 * A set of Particles is a discrete representation of a probability
 * distribution. Normally the user of the class ParticleFilter has
 * not to care about this class, as it is used only internally by
 * ParticleFilter.
 * @author Niklas Fiedler
 * @author Stephan Wirth
 * @see particle_filter
 */
template <class StateType>
class Particle {
 public:
  /**
   * This constructor assigns the given state to the member m_State
   * and the given weight to the member m_Weight.
   * @param state The initial state of the particle
   * @param weight The initial weight of the particle
   */
  Particle<StateType>(const StateType& state, double weight);

  /**
   * The destructor is empty.
   */
  virtual ~Particle();

  /**
   * @return reference to the state of the particle
   */
  inline const StateType& getState() const;

  /**
   * Sets a new state (assignment operator is used, be sure it works
   * for StateType!).
   * @param newState a new state for the particle.
   */
  inline void setState(const StateType& newState);

  /**
   * @return the normalized weight
   */
  inline double getWeight() const;

  /**
   * @return the raw weight
   */
  inline double getWeightUnnormalized() const;

  /**
   * Sets a new weight
   * @param newWeight the new weight
   */
  inline void setWeight(double newWeight);

  /**
   * Sets the normalization factor
   * @param newWeight the new weight
   */
  inline void setNormalization(double normalization);

  bool is_explorer_;

 private:
  // make ParticleFilter a friend that can have non-const access
  // to m_State
  template <class T>
  friend class ParticleFilter;

  // Stores the state of the particle.
  StateType m_State;

  // Stores the importance factor (=weight) of the particle.
  double m_Weight;

  // Stores the normalization
  double m_normalization = 1;
};

template <class StateType>
Particle<StateType>::Particle(const StateType& state, double weight)
    : is_explorer_(false), m_State(state), m_Weight(weight) {}

template <class StateType>
Particle<StateType>::~Particle<StateType>() {}

template <class StateType>
const StateType& Particle<StateType>::getState() const {
  return m_State;
}

template <class StateType>
void Particle<StateType>::setState(const StateType& newState) {
  m_State = newState;
}

template <class StateType>
double Particle<StateType>::getWeight() const {
  return m_Weight * m_normalization;
}

template <class StateType>
double Particle<StateType>::getWeightUnnormalized() const {
  return m_Weight;
}

template <class StateType>
void Particle<StateType>::setWeight(double newWeight) {
  m_Weight = newWeight;
}

template <class StateType>
void Particle<StateType>::setNormalization(double normalization) {
  m_normalization = normalization;
}
}  // namespace particle_filter

#endif  // PARTICLE_HPP
