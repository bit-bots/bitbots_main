#ifndef BITBOTS_SPLINES_ABSTRACTENGINE_H
#define BITBOTS_SPLINES_ABSTRACTENGINE_H

#include <vector>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_splines/SmoothSpline.hpp>

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

template<class Positions, class Goals>
class AbstractEngine {
public:
    /**
     * Do one iteration of spline-progress-updating. This means that whenever update() is called,
     *      new position goals are retrieved from previously calculated splines and returned.
     * @param dt Passed time delta between last call to update() and now. Measured in seconds
     * @return New spline positions
     */
    virtual const Positions update(double dt) = 0;
    virtual void set_goals(const Goals& goals) = 0;
    virtual void reset() = 0;
    virtual const Trajectories get_splines() const = 0;
    /**
     * Returns the percentage of the spline that has already been returned.
     */
    virtual int get_percent_done() const = 0;

    /* Visualisierungszeug */
};

#endif //BITBOTS_SPLINES_ABSTRACTENGINE_H
