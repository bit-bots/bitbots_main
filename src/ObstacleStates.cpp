#include <bitbots_world_model/ObstacleStates.h>

ObstacleState::ObstacleState() :
    xPos_(0.0),
    yPos_(0.0) {}

ObstacleState::ObstacleState(float x, float y) :
    xPos_(x),
    yPos_(y) {}

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

ObstacleStateO::~ObstacleStateO() {}

void ObstacleStateO::setOrientation(float o) {
    orientation_ = o;
}

float ObstacleStateO::getOrientation() const {
    return orientation_;
}

