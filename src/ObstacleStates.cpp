#include <bitbots_world_model/ObstacleStates.h>

ObstacleState::ObstacleState() :
    xPos_(0.0),
    yPos_(0.0) {}

ObstacleState::ObstacleState(float x, float y) :
    xPos_(x),
    yPos_(y) {}

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


ObstacleStateW::ObstacleStateW() :
    ObstacleState(),
    width_(0.0) {}

ObstacleStateW::ObstacleStateW(float x, float y, float w) :
    ObstacleState(x, y),
    width_(w) {}

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


ObstacleStateO::ObstacleStateO() :
    ObstacleState(),
    orientation_(0.0) {}

ObstacleStateO::ObstacleStateO(float x, float y, float o) :
    ObstacleState(x, y),
    orientation_(o) {}

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

