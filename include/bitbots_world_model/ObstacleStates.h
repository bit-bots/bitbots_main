#ifndef OBSTACLE_STATES
#define OBSTACLE_STATES
#include <cmath>

#include <ros/ros.h>
#include <libPF/ParticleFilter.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <bitbots_image_transformer/PixelRelative.h>

class PositionState
{
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

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<PositionState>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    static visualization_msgs::Marker renderMarker(PositionState particle_state, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    double calcDistance(const PositionState& state) const;

    double calcDistance(const bitbots_image_transformer::PixelRelative &pixel) const;

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

class PositionStateW : public PositionState
{
public:
    PositionStateW();
    PositionStateW(float x, float y, float w);

    PositionStateW operator*(float factor) const;

    PositionStateW& operator+=(const PositionStateW& other);

    PositionStateW& operator=(const PositionStateW& other);
    ~PositionStateW();

    float getWidth() const;

    void setWidth(float t);

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<PositionStateW>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    double calcDistance(const PositionStateW& state) const;

private:

    float width_;
};


/*
 * PoseState - the obstacle state with orientation
 */

class PoseState : public PositionState
{
public:
    PoseState();
    PoseState(float x, float y, float o);

    PoseState operator*(float factor) const;

    PoseState& operator+=(const PoseState& other);

    PoseState& operator=(const PoseState& other);
    ~PoseState();

    float getOrientation() const;

    void setOrientation(float t);

    // a marker is not useful for visualization here.
    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<PositionState>::ParticleList& particle_list, std_msgs::ColorRGBA color, std::string n_space) = delete;

    double calcDistance(const PoseState& state) const;

private:

    float orientation_;
};

#endif

