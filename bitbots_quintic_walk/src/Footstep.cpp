/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/Footstep.hpp"

namespace bitbots_quintic_walk {

Footstep::Footstep(
    double foot_distance,
    bool is_left_support_foot) :
    _footDistance(foot_distance),
    _isLeftSupportFoot(is_left_support_foot),
    support_to_last_(),
    support_to_next_(),
    left_in_world_(),
    right_in_world_() {
  if (foot_distance <= 0.0) {
    throw std::logic_error(
        "Footstep invalid distance");
  }

  //State initialization
  left_in_world_.setZero();
  right_in_world_.setZero();
  reset(_isLeftSupportFoot);
}

void Footstep::setFootDistance(double foot_distance) {
  _footDistance = foot_distance;
}

double Footstep::getFootDistance() {
  return _footDistance;
}

void Footstep::reset(bool is_left_support_foot) {
  _isLeftSupportFoot = is_left_support_foot;
  support_to_last_.x() = 0.0;
  if (_isLeftSupportFoot) {
    support_to_last_.y() = -_footDistance;
  } else {
    support_to_last_.y() = _footDistance;
  }
  support_to_last_.z() = 0.0;
  support_to_next_ = support_to_last_;
}

void Footstep::resetInWorld(bool is_left_support_foot) {
  if (_isLeftSupportFoot) {
    right_in_world_.y() = -_footDistance;
  } else {
    left_in_world_.y() = _footDistance;
  }
}

bool Footstep::isLeftSupport() const {
  return _isLeftSupportFoot;
}
const Eigen::Vector3d &Footstep::getLast() const {
  return support_to_last_;
}
const Eigen::Vector3d &Footstep::getNext() const {
  return support_to_next_;
}
const Eigen::Vector3d &Footstep::getLeft() const {
  return left_in_world_;
}
const Eigen::Vector3d &Footstep::getRight() const {
  return right_in_world_;
}

void Footstep::stepFromSupport(const Eigen::Vector3d &diff) {
  //Update relative diff from support foot
  support_to_last_ = diffInv(support_to_next_);
  support_to_next_ = diff;
  //Update world integrated position
  if (_isLeftSupportFoot) {
    left_in_world_ = poseAdd(right_in_world_, diff);
  } else {
    right_in_world_ = poseAdd(left_in_world_, diff);
  }
  //Update current support foot
  _isLeftSupportFoot = !_isLeftSupportFoot;
}

void Footstep::stepFromOrders(const Eigen::Vector3d &diff) {
  //Compute step diff in next support foot frame
  Eigen::Vector3d tmp_diff = Eigen::Vector3d::Zero();
  //No change in forward step
  tmp_diff.x() = diff.x();
  //Add lateral foot offset
  if (_isLeftSupportFoot) {
    tmp_diff.y() += _footDistance;
  } else {
    tmp_diff.y() -= _footDistance;
  }
  //Allow lateral step only on external foot
  //(internal foot will return to zero pose)
  if (
      (_isLeftSupportFoot && diff.y() > 0.0) ||
          (!_isLeftSupportFoot && diff.y() < 0.0)
      ) {
    tmp_diff.y() += diff.y();
  }
  //No change in turn (in order to
  //rotate arroud trunk center)
  tmp_diff.z() = diff.z();

  //Make the step
  stepFromSupport(tmp_diff);
}

Eigen::Vector3d Footstep::poseAdd(
    const Eigen::Vector3d &pose,
    const Eigen::Vector3d &diff) const {
  Eigen::Vector3d tmp_pose = pose;
  double aa = pose.z();
  tmp_pose.x() += diff.x()*std::cos(aa) - diff.y()*std::sin(aa);
  tmp_pose.y() += diff.x()*std::sin(aa) + diff.y()*std::cos(aa);
  tmp_pose.z() = bitbots_splines::AngleBound(tmp_pose.z() + diff.z());

  return tmp_pose;
}

Eigen::Vector3d Footstep::diffInv(
    const Eigen::Vector3d &diff) const {
  Eigen::Vector3d tmp_diff;
  double aa = -diff.z();
  tmp_diff.x() = -diff.x()*std::cos(aa) + diff.y()*std::sin(aa);
  tmp_diff.y() = -diff.x()*std::sin(aa) - diff.y()*std::cos(aa);
  tmp_diff.z() = -diff.z();

  return tmp_diff;
}

}

