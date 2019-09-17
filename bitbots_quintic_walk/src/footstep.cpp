/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_quintic_walk/footstep.h"

namespace bitbots_quintic_walk {

Footstep::Footstep(
    double foot_distance,
    bool is_left_support_foot) :
    foot_distance_(foot_distance),
    is_left_support_foot_(is_left_support_foot),
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
  reset(is_left_support_foot_);
}

void Footstep::setfoot_distance(double foot_distance) {
  foot_distance_ = foot_distance;
}

double Footstep::getfoot_distance() {
  return foot_distance_;
}

void Footstep::reset(bool is_left_support_foot) {
  is_left_support_foot_ = is_left_support_foot;
  support_to_last_.x() = 0.0;
  if (is_left_support_foot_) {
    support_to_last_.y() = -foot_distance_;
  } else {
    support_to_last_.y() = foot_distance_;
  }
  support_to_last_.z() = 0.0;
  support_to_next_ = support_to_last_;
}

void Footstep::resetInWorld(bool is_left_support_foot) {
  if (is_left_support_foot_) {
    right_in_world_.y() = -foot_distance_;
  } else {
    left_in_world_.y() = foot_distance_;
  }
}

bool Footstep::isLeftSupport() const {
  return is_left_support_foot_;
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
  if (is_left_support_foot_) {
    left_in_world_ = poseAdd(right_in_world_, diff);
  } else {
    right_in_world_ = poseAdd(left_in_world_, diff);
  }
  //Update current support foot
  is_left_support_foot_ = !is_left_support_foot_;
}

void Footstep::stepFromOrders(const Eigen::Vector3d &diff) {
  //Compute step diff in next support foot frame
  Eigen::Vector3d tmp_diff = Eigen::Vector3d::Zero();
  //No change in forward step
  tmp_diff.x() = diff.x();
  //Add lateral foot offset
  if (is_left_support_foot_) {
    tmp_diff.y() += foot_distance_;
  } else {
    tmp_diff.y() -= foot_distance_;
  }
  //Allow lateral step only on external foot
  //(internal foot will return to zero pose)
  if (
      (is_left_support_foot_ && diff.y() > 0.0) ||
          (!is_left_support_foot_ && diff.y() < 0.0)
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

