#ifndef OBSERVATIONMODELS
#define OBSERVATIONMODELS

#include <vector>
#include <algorithm>
#include <particle_filter/ObservationModel.h>

#include <bitbots_world_model/ObstacleStates.h>
#include <humanoid_league_msgs/PixelsRelative.h>
#include <humanoid_league_msgs/PixelRelative.h>


/**
 * @class LocalObstacleObservationModel
 *
 * @brief Observation model that measures a PositionStateW
 *
 * @author Niklas Fiedler
 *
 */
class LocalObstacleObservationModel
        : public particle_filter::ObservationModel<PositionStateW> {
public:
    /**
     * empty
     */
    LocalObstacleObservationModel();

    /**
     * empty
     */
    ~LocalObstacleObservationModel();

    /**
     * Measures the weight of a PositionStateW particle and returns it.
     *
     * @param state Reference to the particle that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionStateW& state) const;

    /**
     * Setting the measurements which are used as a base to weight the
     * particles.
     *
     * @param measurement vector of PositionStateW used as mesurement
     */
    void set_measurement(std::vector<PositionStateW> measurement);

    /**
     * Sets the minimal weight a particle can have. In case the measured weight
     * is below that value, it is set to it.
     *
     * @param min_weight the minimal weight a particle can have
     */
    void set_min_weight(double min_weight);

    /**
     * Returns the minimal weight a particle can have.
     *
     * @return the minimal weight a particle can have
     */
    double get_min_weight() const;

    /**
     * Clears the list of measurements.
     */
    void clear_measurement();

    /**
     * Returns true if the vector of measurements is not empty.
     *
     * @return true if the vector of measurements is not empty
     */
    bool measurements_available();

protected:
private:
    std::vector<PositionStateW> last_measurement_;

    double min_weight_;
};

class LocalRobotObservationModel
        : public particle_filter::ObservationModel<PositionState> {
public:
    /**
     * empty
     */
    LocalRobotObservationModel();

    /**
     * empty
     */
    ~LocalRobotObservationModel();

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionState& state) const;

    void set_measurement(std::vector<PositionState> measurement);

    /**
     * Sets the minimal weight a particle can have. In case the measured weight
     * is below that value, it is set to it.
     *
     * @param min_weight the minimal weight a particle can have
     */
    void set_min_weight(double min_weight);

    /**
     * Returns the minimal weight a particle can have.
     *
     * @return the minimal weight a particle can have
     */
    double get_min_weight() const;

    /**
     * Clears the list of measurements.
     */
    void clear_measurement();

    /**
     * Returns true if the vector of measurements is not empty.
     *
     * @return true if the vector of measurements is not empty
     */
    bool measurements_available();

protected:
private:
    std::vector<PositionState> last_measurement_;

    double min_weight_;
};

class GlobalRobotObservationModel
        : public particle_filter::ObservationModel<PositionState> {
public:
    /**
     * empty
     */
    GlobalRobotObservationModel();

    /**
     * empty
     */
    ~GlobalRobotObservationModel();

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionState& state) const;

    void set_measurement(std::vector<PositionState> measurement);

    /**
     * Sets the minimal weight a particle can have. In case the measured weight
     * is below that value, it is set to it.
     *
     * @param min_weight the minimal weight a particle can have
     */
    void set_min_weight(double min_weight);

    /**
     * Returns the minimal weight a particle can have.
     *
     * @return the minimal weight a particle can have
     */
    double get_min_weight() const;

    /**
     * Clears the list of measurements.
     */
    void clear_measurement();

    /**
     * Returns true if the vector of measurements is not empty.
     *
     * @return true if the vector of measurements is not empty
     */
    bool measurements_available();

protected:
private:
    std::vector<PositionState> last_measurement_;

    double min_weight_;
};

class GlobalBallObservationModel
        : public particle_filter::ObservationModel<PositionState> {
public:
    /**
     * empty
     */
    GlobalBallObservationModel();

    /**
     * empty
     */
    ~GlobalBallObservationModel();

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionState& state) const;

    void set_measurement(std::vector<PositionState> measurement);

    /**
     * Sets the minimal weight a particle can have. In case the measured weight
     * is below that value, it is set to it.
     *
     * @param min_weight the minimal weight a particle can have
     */
    void set_min_weight(double min_weight);

    /**
     * Returns the minimal weight a particle can have.
     *
     * @return the minimal weight a particle can have
     */
    double get_min_weight() const;

    /**
     * Clears the list of measurements.
     */
    void clear_measurement();

    /**
     * Returns true if the vector of measurements is not empty.
     *
     * @return true if the vector of measurements is not empty
     */
    bool measurements_available();

protected:
private:
    std::vector<PositionState> last_measurement_;

    double min_weight_;
};

class LocalFcnnObservationModel
        : public particle_filter::ObservationModel<PositionState> {
public:
    /**
     * empty
     */
    LocalFcnnObservationModel();

    /**
     * empty
     */
    ~LocalFcnnObservationModel();

    typedef struct {
        double distance;
        double weight;
    } WeightedMeasurement;

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionState& state) const;

    void set_measurement(humanoid_league_msgs::PixelsRelative measurement);

    /**
     * Setting the number of measurements in the environment of a particle which
     * has to be considered in the weighting of a particle.
     *
     * @param k the number of measurements in the environment of a particle
     * which has to be considered
     */
    void set_k(int k);

    /**
     * Sets the minimal weight a particle can have. In case the measured weight
     * is below that value, it is set to it.
     *
     * @param min_weight the minimal weight a particle can have
     */
    void set_min_weight(double min_weight);

    /**
     * Returns the minimal weight a particle can have.
     *
     * @return the minimal weight a particle can have
     */
    double get_min_weight() const;

    /**
     * Clears the list of measurements.
     */
    void clear_measurement();

    /**
     * Returns true if the vector of measurements is not empty.
     *
     * @return true if the vector of measurements is not empty
     */
    bool measurements_available();

protected:
private:
    std::vector<humanoid_league_msgs::PixelRelative> last_measurement_;

    double min_weight_;
    int k_;  // count of elements considered as "near"
};

#endif
