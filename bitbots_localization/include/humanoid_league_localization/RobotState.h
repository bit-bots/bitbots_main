//
// Created by judith on 09.03.19.
//

#ifndef HUMANOID_LEAGUE_LOCALIZATION_ROBOTSTATE_H
#define HUMANOID_LEAGUE_LOCALIZATION_ROBOTSTATE_H

#include <cmath>
#include <particle_filter/ParticleFilter.h>
#include <vector>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

/**
* @class CarState
* @brief Sample state for a particle filter that simulates a car.
*
* This state has the following parameters:
* @li <b>xpos</b> the x-Position of the car
* @li <b>ypos</b> the y-Position of the car
* @li <b>theta</b> the orientation of the car (in radiants)
* @li <b>speed</b> the forward speed of the car
* @li <b>rotationSpeed</b> the speed with which the car rotates.
*/
class RobotState
{
public:
    RobotState();
    RobotState(double x, double y, double T);
    ~RobotState();

    RobotState operator*(float factor) const;

    RobotState& operator+=(const RobotState& other);


    double getXPos() const;

    double getYPos() const;

    double getTheta() const;

    double getSinTheta() const;

    double getCosTheta() const;

    void setXPos(double x);

    void setYPos(double y);

    void setTheta(double t);

    void setSinTheta(double t);

    void setCosTheta(double t);

    double calcDistance(const RobotState& state) const;

    static void convertParticleListToEigen(const std::vector<particle_filter::Particle<RobotState>*>& particle_list,
                                                Eigen::MatrixXd& matrix,
                                                const bool ignore_explorers);

    bool is_explorer_;

    visualization_msgs::Marker renderMarker(std::string n_space, std::string frame, ros::Duration lifetime, std_msgs::ColorRGBA color) const;


private:

    double m_XPos;
    double m_YPos;
    double m_SinTheta;
    double m_CosTheta;
};

#endif //HUMANOID_LEAGUE_LOCALIZATION_ROBOTSTATE_H
