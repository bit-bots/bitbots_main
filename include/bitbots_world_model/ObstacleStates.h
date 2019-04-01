#ifndef OBSTACLE_STATES
#define OBSTACLE_STATES

#include <omp.h>

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Core>
#include <particle_filter/ParticleFilter.h>
#include <particle_filter/gaussian_mixture_model.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <humanoid_league_msgs/PixelRelative.h>

class PositionState {
public:
    PositionState();
    PositionState(float x, float y);
    ~PositionState();

    PositionState operator*(float factor) const;

    PositionState& operator=(const PositionState& other);

    PositionState& operator+=(const PositionState& other);

    float getXPos() const;

    void setXPos(float x);

    float getYPos() const;

    void setYPos(float y);

    static visualization_msgs::Marker renderPointsMarker(
            particle_filter::ParticleFilter<PositionState>::ParticleList&
                    particle_list,
            std::string n_space,
            std::string frame,
            ros::Duration lifetime,
            std_msgs::ColorRGBA color);

    static visualization_msgs::Marker renderMarker(PositionState particle_state,
            std_msgs::ColorRGBA color,
            ros::Duration lifetime,
            std::string n_space);

    double calcDistance(const PositionState& state) const;

    double calcDistance(const humanoid_league_msgs::PixelRelative& pixel) const;

    static void convertParticleListToEigen(
            const std::vector<particle_filter::Particle<PositionState>*>&
                    particle_list,
            Eigen::MatrixXd& matrix,
            const bool ignore_explorers);

    // float getTheta() const;

    // void setTheta(float t);

    // float getSpeed() const;

    // void setSpeed(float s);

    // float getRotationSpeed() const;

    // void setRotationSpeed(float s);

    bool is_explorer_;


protected:
    float xPos_;
    float yPos_;
};

/*
 * PositionStateW - the obstacle state with width
 */

class PositionStateW : public PositionState {
public:
    PositionStateW();
    PositionStateW(float x, float y, float w);

    PositionStateW operator*(float factor) const;

    PositionStateW& operator+=(const PositionStateW& other);

    PositionStateW& operator=(const PositionStateW& other);
    ~PositionStateW();

    float getWidth() const;

    void setWidth(float t);

    static visualization_msgs::Marker renderPointsMarker(
            particle_filter::ParticleFilter<PositionStateW>::ParticleList&
                    particle_list,
            std::string n_space,
            std::string frame,
            ros::Duration lifetime,
            std_msgs::ColorRGBA color);

    double calcDistance(const PositionStateW& state) const;

    static void convertParticleListToEigen(
            const std::vector<particle_filter::Particle<PositionStateW>*>&
                    particle_list,
            Eigen::MatrixXd& matrix,
            const bool ignore_explorers);

private:
    float width_;
};


/*
 * PoseState - the obstacle state with orientation
 */

class PoseState : public PositionState {
public:
    PoseState();
    PoseState(float x, float y, float o);

    PoseState operator*(float factor) const;

    PoseState& operator+=(const PoseState& other);

    PoseState& operator=(const PoseState& other);
    ~PoseState();

    float getOrientation() const;

    void setOrientation(float t);

    // a points marker is not useful for visualization here.
    static visualization_msgs::Marker
    renderPointsMarker(particle_filter::ParticleFilter<PoseState>::ParticleList&
                               particle_list,
            std::string n_space,
            std::string frame,
            ros::Duration lifetime,
            std_msgs::ColorRGBA color);

    double calcDistance(const PoseState& state) const;

    static void convertParticleListToEigen(
            const std::vector<particle_filter::Particle<PoseState>*>&
                    particle_list,
            Eigen::MatrixXd& matrix,
            const bool ignore_explorers);

private:
    float orientation_;
};

#endif
