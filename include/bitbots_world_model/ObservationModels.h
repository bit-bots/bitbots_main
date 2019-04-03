#ifndef OBSERVATIONMODELS
#define OBSERVATIONMODELS

#include <vector>
#include <algorithm>
#include <particle_filter/ObservationModel.h>

#include <bitbots_world_model/ObstacleStates.h>
#include <humanoid_league_msgs/PixelsRelative.h>
#include <humanoid_league_msgs/PixelRelative.h>


/**
 * @class MyObservationModel
 *
 * @brief Observation model that measures a MyState.
 *
 * The measurement is made according to the formula
 * \f[
 *     w = \frac{1}{|x^2 - 2|}
 * \f]
 * where x is the variable of MyState and w is the weight that is returned.
 * This is a measure for the distance from x to the squareroot of two. If
 * The distance is low, the returned weight is high.
 *
 * @author Stephan Wirth
 *
 * @brief Test class for ParticleFilter.
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
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const PositionStateW& state) const;

    void set_measurement(std::vector<PositionStateW> measurement);

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

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

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

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

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

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

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

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

    void set_min_weight(double min_weight);

    void set_k(int k);

    double get_min_weight() const;

    void clear_measurement();

    bool measurements_available();

protected:
private:
    std::vector<humanoid_league_msgs::PixelRelative> last_measurement_;

    double min_weight_;
    int k_;  // count of elements considered as "near"
};

#endif
