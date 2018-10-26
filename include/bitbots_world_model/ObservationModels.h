#ifndef OBSERVATIONMODELS
#define OBSERVATIONMODELS

#include <vector>
#include <algorithm>
#include <bitbots_world_model/ObstacleStates.h>
#include <libPF/ObservationModel.h>


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
class LocalObstacleObservationModel : public libPF::ObservationModel<ObstacleStateW> {

  public:

    /**
     * empty
     */
    LocalObstacleObservationModel ();

    /**
     * empty
     */
    ~LocalObstacleObservationModel ();

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const ObstacleStateW& state) const;

    void set_measurement(std::vector<ObstacleStateW> measurement);

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

    bool measurements_available();

  protected:

  private:

    std::vector<ObstacleStateW> last_measurement_;

    double min_weight_;

};

class LocalRobotObservationModel : public libPF::ObservationModel<ObstacleState> {

  public:

    /**
     * empty
     */
    LocalRobotObservationModel ();

    /**
     * empty
     */
    ~LocalRobotObservationModel ();

    /**
     *
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
    double measure(const ObstacleState& state) const;

    void set_measurement(std::vector<ObstacleState> measurement);

    void set_min_weight(double min_weight);

    double get_min_weight() const;

    void clear_measurement();

    bool measurements_available();

  protected:

  private:

    std::vector<ObstacleState> last_measurement_;

    double min_weight_;

};

#endif

