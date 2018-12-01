#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H
#include <iostream>
#include <ctime> // for time measurement
#include <cassert>
#include <limits>
#include <memory>

#include "libPF/ObservationModel.h"
#include "libPF/MovementModel.h"
#include "libPF/ResamplingStrategy.h"
#include "libPF/ImportanceResampling.h"
#include "libPF/CompareParticleWeights.h"
#include "libPF/Particle.h"
#include "libPF/StateDistribution.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>

namespace libPF
{

/**
 * @class ParticleFilter
 *
 * @author Stephan Wirth
 *
 * @brief (Templated) class that defines a particle filter.
 *
 * A particle filter is a descrete method to describe and compute with a probability distribution.
 * In other words, it is an implementation of a recursive Bayesian filter by Monte Carlo Methods.
 * The sequential importance sampling (SIS) used is also known as bootstrap filtering, the
 * condensation algorithm, particle filtering, interacting particle approximations and survival
 * of the fittest.
 * This template class implements the basic methods for a particle filter.
 * The changeable parts of the particle filter are implemented using the strategy pattern. If you
 * don't know what it is, you should read something about it, as it is used in many methods here.
 *
 * The following strategies are used by ParticleFilter, all of them can be switched at runtime.
 * @li ObservationModel defines how a state can be evaluated (weighted)
 * @li MovementModel defines how a state will be propagated during time
 * @li ResamplingStrategy defines how resampling occurs (see ImportanceResampling for the default implementation)
 *
 *
 * You must do the following to use the particle filter:
 * @li Create a class for the state that you want to track with the ParticleFilter.
 *     Let's name your state MyState.  The state has to implement the operator
 *     @code
 *       operator=(const MyState& other);
 *     @endcode
 *     because a particle filter copies the "fittest" states in a resampling step.
 *     If you want to use the methods getBestXPercentEstimate() or getMmseEstimate(),
 *     your state has to implement two more operators:
 *     @code
 *       MyState operator*(float factor) const;
 *     @endcode
 *     and
 *     @code
 *       MyState& operator+=(const MyState& other);
 *     @endcode
 *     These operators allow the ParticleFilter to compute the weighted average
 *     of a set of states.
 *     Instead of creating your own state, you may use a basic datatype as state,
 *     for example to create a <tt>ParticleFilter<float></tt> is perfectly possible.
 * @li The next step is to create an observation strategy MyObservationModel for your state, deriving
 *     it from ObservationModel<MyState> and implementing the method
 *     ObservationModel::measure().
 * @li Create a movement strategy MyMovementModel for your state, deriving it from
 *     MovementModel<MyState> and implement the methods MovementModel::diffuse()
 *     and MovementModel::drift().
 *
 * After that you can use the particle filter this way:
 * @code
 *   int numOfParticles = 500;
 *   MyMovementModel mm;              // create movement strategy
 *   MyObservationModel om;           // create observation strategy
 *   ParticleFilter<MyState> pf(numOfParticles, &om, &mm); // create filter
 *
 *   // run the filter loop
 *   bool doFilter = true;
 *   while (doFilter) {
 *     // update your observation model here
 *     // ...
 *     // update your movement model here (if necessary)
 *     // ...
 *
 *     // run one filter step, the filter uses the
 *     // observation model and the movement model
 *     // that were given in the constructor
 *     pf->filter();
 *
 *     // retrieve the best state and
 *     // do something with the result
 *     std::cout << pf->getBestState().getVariable1() << std::endl;
 *
 *   }
 * @endcode
 *
 * The ParticleFilter has the following resampling modes:
 * @li RESAMPLE_NEVER skip resampling,
 * @li RESAMPLE_ALWAYS does a resampling in every filter step whenever
 *     filter() is called,
 * @li RESAMPLE_NEFF does a resampling in filter() only if the number of
 *     effective particles falls below the half of the total number of
 *     particles (see getNumEffectiveParticles() for details).
 *
 * The default is RESAMPLE_NEFF.
 * You can switch the mode via setResamplingMode().
 *
 * You have two options to influence the states that are used internally by
 * the particle filter. The first one is to set a prior state:
 * @code
 *   // initialize the filter's states by setting a prior state
 *   MyState priorState;
 *   priorState.setPosition(20, 30);
 *   priorState.setVelocity(1.2, 0);
 *   pf.setPriorState(priorState);
 * @endcode
 * The second option is to use a state distribution that is derived from
 * StateDistribution:
 * @code
 *   // create a distribution
 *   MyStateDistribution distribution;
 *   // draw all states from this distribution
 *   pf.drawAllFromDistribution(distribution);
 * @endcode
 *
 * To traverse the particle list, you may use particleListBegin() and particleListEnd()
 * which return iterators to the beginning and to the end of the list respectively.
 *
 * @see Particle
 * @see ObservationModel
 * @see MovementModel
 * @see ResamplingStrategy
 */

/**
  * Resampling modes.
  */
enum ResamplingMode
{
    /// never resample,
    RESAMPLE_NEVER,
    /// always resample
    RESAMPLE_ALWAYS,
    /// only resample if Neff < numParticles / 2
    RESAMPLE_NEFF
};

template <class StateType>
class ParticleFilter {

  public:

    /**
     * A ParticleList is an array of pointers to Particles.
     */
    typedef std::vector< Particle<StateType>* > ParticleList;

    /**
     * Typedef for an iterator over particles
     */
    typedef typename ParticleList::iterator ParticleIterator;

    /**
     * Typedef for a const iterator over particles
     */
    typedef typename ParticleList::const_iterator ConstParticleIterator;

    /**
     * The constructor allocates the memory for the particle lists and saves
     * <b>pointers</b> to ObservationModel and MovementModel in member variables.
     * Be sure that these objects are valid through the lifetime of ParticleFilter!
     * The default constructor of StateType will be used to create the
     * initial particles.
     * The particle lists will have @a numParticles elements of type StateType.
     * @param numParticles Number of particles for the filter. Has to be greater
     *        than zero.
     * @param os ObservationModel to use for weightening particles
     * @param ms MovementModel to use for propagation of particles
     */
    ParticleFilter<StateType>(unsigned int numParticles, std::shared_ptr<ObservationModel<StateType>> os, std::shared_ptr<MovementModel<StateType>> ms);

    /**
     * The destructor releases the particle lists.
     */
    virtual ~ParticleFilter();

    /**
     * @return Number of particles used in this filter
     */
    unsigned int numParticles() const;

    /**
     * @param os new observation model
     */
    void setObservationModel(std::shared_ptr<ObservationModel<StateType>>& os);

    /**
     * @return the observation model the particle filter currently uses
     */
    std::shared_ptr<ObservationModel<StateType>> getObservationModel() const;

    /**
     * @param ms new movement model
     */
    void setMovementModel(std::shared_ptr<MovementModel<StateType>>& ms);

    /**
     * @return the movement model the particle filter currently uses
     */
    std::shared_ptr<MovementModel<StateType>> getMovementModel() const;

    /**
     * @param rs new resampling strategy
     */
    void setResamplingStrategy(std::shared_ptr<ResamplingStrategy<StateType>> rs);

    /**
     * @return the resampling strategy the particle filter currently uses
     */
    std::shared_ptr<ResamplingStrategy<StateType>> getResamplingStrategy() const;

    /**
     * Changes the resampling mode
     * @param mode new resampling mode.
     */
    void setResamplingMode(ResamplingMode mode);

    /**
     * @return the currently set resampling mode
     */
    ResamplingMode getResamplingMode() const;

    /**
     * Computes and returns the number of effective particles.
     * @return The estimated number of effective particles according to the formula:
     * \f[
     *     N_{eff} = \frac{1}{\sum_{i=1}^{N_s} (w_k^i)^2}
     * \f]
     */
    unsigned int getNumEffectiveParticles() const;

    /**
     * Sets all particle states to the given state. Useful for integrating a
     * known prior state to begin with tracking.
     * @param priorState State that will be copied to all particles.
     */
    void setPriorState(const StateType& priorState);

    /**
     * Draws all particle states from the given distribution.
     * @param distribution The state distribution to draw the states from.
     */
    void drawAllFromDistribution(const std::shared_ptr<StateDistribution<StateType>>& distribution);

    /**
     * Resets the filter timer. Call this function after pausing the filter
     * to avoid a drift step with a high delta t.
     */
    void resetTimer();

    /**
     * @return Pointer to the particle that has the highest weight.
     */
    const Particle<StateType>* getBestParticle() const;

    /**
     * @return State that is carried by the particle with heighest weight.
     */
    const StateType& getBestState() const;

    /**
     * @param i Particle index
     * @return weight of Particle i.
     */
    double getWeight(unsigned int i) const;

    /**
     * Performs an entire filter procedure.
     * filter() also saves the last filter time to be able to compute the
     * appropriate dt for the drift step. To reset the time manually, call
     * resetTimer().
     * The functions resample(),
     * drift(), diffuse() and measure() are called.
     * @param dt time interval to use for filtering. If negative, the time interval
     *        will be calculated (time since last call of filter())
     */
    void filter(double dt = -1.0);

    /**
     * Returns a pointer to a particle with a given index.
     * @param particleNo Index of requested particle
     * @return Pointer to particle with index particleNo
     */
    const Particle<StateType>* getParticle(unsigned int particleNo) const;

    /**
     * Returns a const reference to the state of particle with given index.
     * @param particleNo Index of particle
     * @return Pointer to the state of particle at index particleNo.
     */
    const StateType& getState(unsigned int particleNo) const;

    /**
     * Returns the "mean" state, i.e. the sum of the weighted states. You can use this only if you implemented operator*(double) and
     * operator+=(MyState) in your derived State MyState.
     * @return "mean" state. Best estimation.
     */
    StateType getMmseEstimate() const;

    /**
     * Same as getMmseEstimate(), but uses only the best x% of the particles.
     * @param x percentage of particles to use. Has to be positive and greater
     *        than zero.
     * @return "mean" state of the best x% particles. If x <= 0, the state
     *         with the highest weight is returned.
     */
    StateType getBestXPercentEstimate(float x) const;

    /**
     * This method selects a new set of particles out of an old set according to their weight
     * (importance resampling). The particles from the list m_CurrentList points to are used as source,
     * m_LastList points to the destination list. The pointers m_CurrentList and m_LastList are switched.
     * The higher the weight of a particle, the more particles are drawn (copied) from this particle.
     * The weight remains untouched, because measure() will be called afterwards.
     * This method only works on a sorted m_CurrentList, therefore sort() is called first.
     */
    void resample();

    /**
     * This method drifts the particles (second step of a filter process) using
     * the movement model of the particle filter. dt defines the time interval
     * that has to be used in drifting (in seconds).
     */
    void drift(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular);

    /**
     * This method "diffuses" the particles using the movement model of the particle filter to add a small jitter
     * to the particle states. dt defines the time interval that has to be used in diffusion (in seconds).
     */
    void diffuse(double dt);

    /**
     * This method assigns weights to the particles using the observation model of the particle filter.
     */
    virtual void measure();

    /**
     * Returns an iterator to the particle list's beginning.
     */
    ConstParticleIterator particleListBegin();

    /**
     * Returns an iterator to the end of the particle list.
     */
    ConstParticleIterator particleListEnd();

    visualization_msgs::Marker renderMarker();

    void setMarkerLifetime(ros::Duration lifetime);

    void setMarkerColor(float r, float g, float b, float a);

    void setMarkerColor(std_msgs::ColorRGBA color);

    void setMarkerNamespace(std::string);

  protected:

    /**
     * This method sorts the particles according to their weight. STL's std::sort() is used together with the
     * custom compare function CompareParticleWeights().
     * The particle with the highest weight is at position 0 after calling this function.
     */
    void sort();

    /**
     * This method normalizes the weights of the particles. After calling this function, the sum of the weights of
     * all particles in the current particle list equals 1.0.
     * In this function the sum of all weights of the particles of the current particle list is computed and each
     * weight of each particle is devided through this sum.
     */
    void normalize();

  private:

    // Particle lists.
    // The particles are drawn from m_LastList to m_CurrentList to avoid new and delete commands.
    // In each run, the pointers m_CurrentList and m_LastList are switched in resample().
    ParticleList m_CurrentList;
    ParticleList m_LastList;

    // Stores the number of particles.
    unsigned int m_NumParticles;

    // Holds a pointer to the observation strategy (used for weighting)
    std::shared_ptr<ObservationModel<StateType>> m_ObservationModel;

    // Holds a pointer to the movement strategy (used for moving and diffusing)
    std::shared_ptr<MovementModel<StateType>> m_MovementModel;

    // Stores a pointer to the resampling strategy.
    std::shared_ptr<ResamplingStrategy<StateType>> m_ResamplingStrategy;

    // The default resampling strategy.

    // Stores the last filter time to have the right dt value for drift.
    clock_t m_LastDriftTime;

    // Flag that stores if the filter has run once or not)
    bool m_FirstRun;

    // Stores which resampling mode is set, default is ResamplingMode::RESAMPLE_NEFF
    ResamplingMode m_ResamplingMode;

    std_msgs::ColorRGBA color_;

    ros::Duration marker_lifetime_;

    std::string marker_namespace_;

};

} // end of namespace

// include template implementation
#include "ParticleFilter.hxx"

#endif

