#ifndef OBSTACLE_STATES
#define OBSTACLE_STATES
#include <cmath>

#include <ros/ros.h>
#include <libPF/ParticleFilter.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class ObstacleState
{
public:
    ObstacleState();
    ObstacleState(float x, float y);
    ~ObstacleState();

    // ObstacleState operator*(float factor) const;

    ObstacleState& operator=(const ObstacleState& other);


    float getXPos() const;

    void setXPos(float x);

    float getYPos() const;

    void setYPos(float y);

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleState>::ParticleList& particle_list);

    double calcDistance(ObstacleState& state);

    // float getTheta() const;

    // void setTheta(float t);

    // float getSpeed() const;

    // void setSpeed(float s);

    // float getRotationSpeed() const;

    // void setRotationSpeed(float s);


protected:

    float xPos_;
    float yPos_;
};

/*
 * ObstacleStateW - the obstacle state with width
 */

class ObstacleStateW :  ObstacleState
{
public:
    ObstacleStateW();
    ObstacleStateW(float x, float y, float w);
    ObstacleStateW& operator=(const ObstacleStateW& other);
    ~ObstacleStateW();

    float getWidth() const;

    void setWidth(float t);

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleStateW>::ParticleList& particle_list);

    double calcDistance(ObstacleStateW& state);

private:

    float width_;
};


/*
 * ObstacleStateO - the obstacle state with orientation
 */

class ObstacleStateO :  ObstacleState
{
public:
    ObstacleStateO();
    ObstacleStateO(float x, float y, float o);
    ObstacleStateO& operator=(const ObstacleStateO& other);
    ~ObstacleStateO();

    float getOrientation() const;

    void setOrientation(float t);

    // a marker is not useful for visualization here.
    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleState>::ParticleList& particle_list) = delete;

    double calcDistance(ObstacleStateO& state);

private:

    float orientation_;
};

#endif

