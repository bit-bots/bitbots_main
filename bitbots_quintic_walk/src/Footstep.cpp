/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/Footstep.hpp"

namespace bitbots_quintic_walk {

Footstep::Footstep(
    double footDistance,
    bool isLeftSupportFoot) :
    _footDistance(footDistance),
    _isLeftSupportFoot(isLeftSupportFoot),
    _supportToLast(),
    _supportToNext(),
    _leftInWorld(),
    _rightInWorld()
{
    if (footDistance <= 0.0) {
        throw std::logic_error(
            "Footstep invalid distance");
    }
    
    //State initialization
    _leftInWorld.setZero();
    _rightInWorld.setZero();
    reset(_isLeftSupportFoot);
}
        
void Footstep::setFootDistance(double footDistance)
{
    _footDistance = footDistance;
}

double Footstep::getFootDistance()
{
    return _footDistance;
}

        
void Footstep::reset(bool isLeftSupportFoot)
{
    _isLeftSupportFoot = isLeftSupportFoot;
    _supportToLast.x() = 0.0;
    if (_isLeftSupportFoot) {
        _supportToLast.y() = -_footDistance;        
    } else {
        _supportToLast.y() = _footDistance;
    }
    _supportToLast.z() = 0.0;
    _supportToNext = _supportToLast;
}

void Footstep::resetInWorld(bool isLeftSupportFoot){
    if (_isLeftSupportFoot) {
        _rightInWorld.y() = -_footDistance;        
    } else {
        _leftInWorld.y() = _footDistance;
    }
}
        
bool Footstep::isLeftSupport() const
{
    return _isLeftSupportFoot;
}
const Eigen::Vector3d& Footstep::getLast() const
{
    return _supportToLast;
}
const Eigen::Vector3d& Footstep::getNext() const
{
    return _supportToNext;
}
const Eigen::Vector3d& Footstep::getLeft() const
{
    return _leftInWorld;
}
const Eigen::Vector3d& Footstep::getRight() const
{
    return _rightInWorld;
}
        
void Footstep::stepFromSupport(const Eigen::Vector3d& diff)
{
    //Update relative diff from support foot
    _supportToLast = diffInv(_supportToNext);
    _supportToNext = diff;
    //Update world integrated position
    if (_isLeftSupportFoot) {
        _leftInWorld = poseAdd(_rightInWorld, diff);
    } else {
        _rightInWorld = poseAdd(_leftInWorld, diff);
    }
    //Update current support foot
    _isLeftSupportFoot = !_isLeftSupportFoot;
}

void Footstep::stepFromOrders(const Eigen::Vector3d& diff)
{
    //Compute step diff in next support foot frame
    Eigen::Vector3d tmpDiff = Eigen::Vector3d::Zero();
    //No change in forward step
    tmpDiff.x() = diff.x();
    //Add lateral foot offset
    if (_isLeftSupportFoot) {
        tmpDiff.y() += _footDistance;
    } else {
        tmpDiff.y() -= _footDistance;
    }
    //Allow lateral step only on external foot
    //(internal foot will return to zero pose)
    if (
        (_isLeftSupportFoot && diff.y() > 0.0) ||
        (!_isLeftSupportFoot && diff.y() < 0.0)
    ) {
        tmpDiff.y() += diff.y();
    }
    //No change in turn (in order to 
    //rotate arroud trunk center)
    tmpDiff.z() = diff.z();
    
    //Make the step
    stepFromSupport(tmpDiff);
}


Eigen::Vector3d Footstep::poseAdd(
    const Eigen::Vector3d& pose,
    const Eigen::Vector3d& diff) const
{
    Eigen::Vector3d tmpPose = pose;
    double aa = pose.z();
    tmpPose.x() += diff.x()*std::cos(aa) - diff.y()*std::sin(aa);
    tmpPose.y() += diff.x()*std::sin(aa) + diff.y()*std::cos(aa);
    tmpPose.z() = bitbots_splines::AngleBound(tmpPose.z() + diff.z());

    return tmpPose;
}

Eigen::Vector3d Footstep::diffInv(
    const Eigen::Vector3d& diff) const
{
    Eigen::Vector3d tmpDiff;
    double aa = -diff.z();
    tmpDiff.x() = -diff.x()*std::cos(aa) + diff.y()*std::sin(aa);
    tmpDiff.y() = -diff.x()*std::sin(aa) - diff.y()*std::cos(aa);
    tmpDiff.z() = -diff.z();

    return tmpDiff;
}

}

