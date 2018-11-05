#include <bitbots_world_model/ObstacleStates.h>

ObstacleState::ObstacleState() :
    xPos_(0.0),
    yPos_(0.0) {}

ObstacleState::ObstacleState(float x, float y) :
    xPos_(x),
    yPos_(y) {}

ObstacleState ObstacleState::operator*(float factor) const {
    return(ObstacleState(getXPos() * factor, getYPos() * factor));
}

ObstacleState& ObstacleState::operator+=(const ObstacleState& other) {
    setXPos(getXPos() + other.getXPos());
    setYPos(getYPos() + other.getYPos());
    return *this;
}

ObstacleState& ObstacleState::operator=(const ObstacleState& other) {
  if (this != &other) {
    setXPos(other.getXPos());
    setYPos(other.getYPos());
  }
  return *this;
}

ObstacleState::~ObstacleState() {}

void ObstacleState::setXPos(float x) {
    xPos_ = x;
}

void ObstacleState::setYPos(float y) {
    yPos_ = y;
}

float ObstacleState::getXPos() const {
    return xPos_;
}

float ObstacleState::getYPos() const {
    return yPos_;
}

visualization_msgs::Marker ObstacleState::renderMarker(libPF::ParticleFilter<ObstacleState>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime) {
    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/world";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.pose.orientation.w = 1;
    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.05;
    msg.color = color;
    msg.lifetime = lifetime;
    for (libPF::Particle<ObstacleState> *particle : particle_list) {
        geometry_msgs::Point point_msg;
        point_msg.x = particle->getState().getXPos();
        point_msg.y = particle->getState().getYPos();
        msg.points.push_back(point_msg);
    }
    return msg;
}

double ObstacleState::calcDistance(const ObstacleState& state) const {
    // TODO
    double diff =  std::sqrt(std::pow(getXPos() - state.getXPos(), 2) + std::pow(getYPos() - state.getYPos(), 2));
    if (diff == 0.0) {
        diff = 0.0001;
    }
    return diff;
}

ObstacleStateW::ObstacleStateW() :
    ObstacleState(),
    width_(0.0) {}

ObstacleStateW::ObstacleStateW(float x, float y, float w) :
    ObstacleState(x, y),
    width_(w) {}

ObstacleStateW ObstacleStateW::operator*(float factor) const {
    return(ObstacleStateW(getXPos() * factor, getYPos() * factor, getWidth() * factor));
}

ObstacleStateW& ObstacleStateW::operator+=(const ObstacleStateW& other) {
    setXPos(getXPos() + other.getXPos());
    setYPos(getYPos() + other.getYPos());
    setWidth(getWidth() + other.getWidth());
    return *this;
}

ObstacleStateW& ObstacleStateW::operator=(const ObstacleStateW& other) {
  if (this != &other) {
    setXPos(other.getXPos());
    setYPos(other.getYPos());
    setWidth(other.getWidth());
  }
  return *this;
}

ObstacleStateW::~ObstacleStateW() {}

void ObstacleStateW::setWidth(float w) {
    width_ = w;
}

float ObstacleStateW::getWidth() const {
    return width_;
}

visualization_msgs::Marker ObstacleStateW::renderMarker(libPF::ParticleFilter<ObstacleStateW>::ParticleList& particle_list, std_msgs::ColorRGBA color, ros::Duration lifetime) {
    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/world";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.pose.orientation.w = 1;
    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.05;
    msg.color = color;
    msg.lifetime = lifetime;
    for (libPF::Particle<ObstacleStateW> *particle : particle_list) {
        geometry_msgs::Point point_msg;
        point_msg.x = particle->getState().getXPos();
        point_msg.y = particle->getState().getYPos();
        msg.points.push_back(point_msg);
    }
    return msg;
}

double ObstacleStateW::calcDistance(const ObstacleStateW& state) const {
    // TODO
    double diff =  std::sqrt(std::pow(getXPos() - state.getXPos(), 2) + std::pow(getYPos() - state.getYPos(), 2));
    if (diff == 0.0) {
        diff = 0.0001;
    }
    return diff;
}

ObstacleStateO::ObstacleStateO() :
    ObstacleState(),
    orientation_(0.0) {}

ObstacleStateO::ObstacleStateO(float x, float y, float o) :
    ObstacleState(x, y),
    orientation_(o) {}

ObstacleStateO ObstacleStateO::operator*(float factor) const {
    return(ObstacleStateO(getXPos() * factor, getYPos() * factor, getOrientation() * factor));
}

ObstacleStateO& ObstacleStateO::operator+=(const ObstacleStateO& other) {
    setXPos(getXPos() + other.getXPos());
    setYPos(getYPos() + other.getYPos());
    setOrientation(getOrientation() + other.getOrientation());
    return *this;
}

ObstacleStateO& ObstacleStateO::operator=(const ObstacleStateO& other) {
  if (this != &other) {
    setXPos(other.getXPos());
    setYPos(other.getYPos());
    setOrientation(other.getOrientation());
  }
  return *this;
}

ObstacleStateO::~ObstacleStateO() {}

void ObstacleStateO::setOrientation(float o) {
    orientation_ = o;
}

float ObstacleStateO::getOrientation() const {
    return orientation_;
}

double ObstacleStateO::calcDistance(const ObstacleStateO& state) const {
    // TODO
    double diff =  std::sqrt(std::pow(getXPos() - state.getXPos(), 2) + std::pow(getYPos() - state.getYPos(), 2));
    if (diff == 0.0) {
        diff = 0.0001;
    }
    return diff;
}

