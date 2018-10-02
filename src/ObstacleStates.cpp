#include <bitbots_world_model/ObstacleStates.h>

ObstacleState::ObstacleState() :
    xPos_(0.0),
    yPos_(0.0) {}

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

ObstacleStateO::~ObstacleStateO() {}

void ObstacleStateO::setOrientation(float o) {
    orientation_ = o;
}

float ObstacleStateO::getOrientation() const {
    return orientation_;
}

