#include <bitbots_world_model/RobotStates.h>

RobotState::RobotState() :
    xPos_(0.0),
    yPos_(0.0) {}

RobotState::~RobotState() {}

void RobotState::setXPos(float x) {
    xPos_ = x;
}

void RobotState::setYPos(float y) {
    yPos_ = y;
}

float RobotState::getXPos() const {
    return xPos_;
}

float RobotState::getYPos() const {
    return yPos_;
}


RobotStateW::RobotStateW() :
    RobotState(),
    width_(0.0) {}

RobotStateW::~RobotStateW() {}

void RobotStateW::setWidth(float w) {
    width_ = w;
}

float RobotStateW::getWidth() const {
    return width_;
}


RobotStateO::RobotStateO() :
    RobotState(),
    orientation_(0.0) {}

RobotStateO::~RobotStateO() {}

void RobotStateO::setOrientation(float o) {
    orientation_ = o;
}

float RobotStateO::getOrientation() const {
    return orientation_;
}

