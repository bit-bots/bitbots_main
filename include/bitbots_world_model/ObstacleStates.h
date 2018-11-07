#ifndef OBSTACLE_STATES
#define OBSTACLE_STATES
#include <cmath>

#include <ros/ros.h>
#include <libPF/ParticleFilter.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

class ObstacleState
{
public:
    ObstacleState();
    ObstacleState(float x, float y);
    ~ObstacleState();

    ObstacleState operator*(float factor) const;

    ObstacleState& operator=(const ObstacleState& other);

    ObstacleState& operator+=(const ObstacleState& other);

    float getXPos() const;

    void setXPos(float x);

    float getYPos() const;

    void setYPos(float y);

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleState>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    static visualization_msgs::Marker renderMarker(ObstacleState particle_state, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    double calcDistance(const ObstacleState& state) const;

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
 * ObstacleStateW - the obstacle state with width
 */

class ObstacleStateW : public ObstacleState
{
public:
    ObstacleStateW();
    ObstacleStateW(float x, float y, float w);

    ObstacleStateW operator*(float factor) const;

    ObstacleStateW& operator+=(const ObstacleStateW& other);

    ObstacleStateW& operator=(const ObstacleStateW& other);
    ~ObstacleStateW();

    float getWidth() const;

    void setWidth(float t);

    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleStateW>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime, std::string n_space);

    double calcDistance(const ObstacleStateW& state) const;

private:

    float width_;
};


/*
 * ObstacleStateO - the obstacle state with orientation
 */

class ObstacleStateO : public ObstacleState
{
public:
    ObstacleStateO();
    ObstacleStateO(float x, float y, float o);

    ObstacleStateO operator*(float factor) const;

    ObstacleStateO& operator+=(const ObstacleStateO& other);

    ObstacleStateO& operator=(const ObstacleStateO& other);
    ~ObstacleStateO();

    float getOrientation() const;

    void setOrientation(float t);

    // a marker is not useful for visualization here.
    static visualization_msgs::Marker renderMarker(libPF::ParticleFilter<ObstacleState>::ParticleList& particle_list, std_msgs::ColorRGBA color, std::string n_space) = delete;

    double calcDistance(const ObstacleStateO& state) const;

private:

    float orientation_;
};

#endif

