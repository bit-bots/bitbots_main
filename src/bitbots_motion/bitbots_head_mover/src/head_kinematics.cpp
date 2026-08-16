#include <bitbots_head_mover/head_kinematics.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <vector>

namespace bitbots_head_mover {

namespace {

/// Convert a KDL frame to an Eigen transform.
Eigen::Isometry3d toEigen(const KDL::Frame& frame) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      result.linear()(row, col) = frame.M(row, col);
    }
    result.translation()(row) = frame.p(row);
  }
  return result;
}

/// Read the position limits of a joint from a parsed robot description.
///
/// Returns false if the joint does not exist or does not declare limits, which
/// is the case for continuous and fixed joints.
bool readJointLimit(const urdf::Model& model, const std::string& joint_name, JointLimit& limit) {
  const auto joint = model.getJoint(joint_name);
  if (!joint || !joint->limits) {
    return false;
  }
  limit.lower = joint->limits->lower;
  limit.upper = joint->limits->upper;
  return true;
}

}  // namespace

std::unique_ptr<HeadKinematics> HeadKinematics::fromUrdf(const std::string& urdf, const HeadChainConfig& config) {
  urdf::Model model;
  if (!model.initString(urdf)) {
    return nullptr;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    return nullptr;
  }

  // std::unique_ptr cannot use make_unique here because the constructor is private
  auto kinematics = std::unique_ptr<HeadKinematics>(new HeadKinematics());

  if (!tree.getChain(config.root_link, config.tip_link, kinematics->chain_)) {
    return nullptr;
  }

  // Locate the head joints among the movable joints of the chain. Fixed joints
  // do not get an entry in the joint array the solver is fed with, so the
  // indices have to be counted rather than derived from the segment order.
  bool found_yaw = false;
  bool found_pitch = false;
  unsigned int movable_index = 0;
  for (const auto& segment : kinematics->chain_.segments) {
    const KDL::Joint& joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    if (joint.getName() == config.yaw_joint) {
      kinematics->yaw_index_ = movable_index;
      found_yaw = true;
    } else if (joint.getName() == config.pitch_joint) {
      kinematics->pitch_index_ = movable_index;
      found_pitch = true;
    }
    movable_index++;
  }

  // Anything else in the chain would move the camera without us knowing about
  // it, which would silently invalidate every sampled camera pose
  if (!found_yaw || !found_pitch || movable_index != 2) {
    return nullptr;
  }

  if (!readJointLimit(model, config.yaw_joint, kinematics->urdf_limits_.yaw) ||
      !readJointLimit(model, config.pitch_joint, kinematics->urdf_limits_.pitch)) {
    return nullptr;
  }

  // The solver refers to the chain, so it has to be built after the chain
  // reached its final location inside the object
  kinematics->solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kinematics->chain_);
  kinematics->joints_.resize(kinematics->chain_.getNrOfJoints());

  return kinematics;
}

std::optional<Eigen::Isometry3d> HeadKinematics::cameraPose(const HeadPosition& position) const {
  joints_(yaw_index_) = position.yaw;
  joints_(pitch_index_) = position.pitch;

  KDL::Frame frame;
  if (solver_->JntToCart(joints_, frame) < 0) {
    // Report the failure instead of substituting a pose. Scoring a made up
    // camera pose would look exactly like scoring a real one.
    return std::nullopt;
  }

  return toEigen(frame) * calibration_;
}

}  // namespace bitbots_head_mover
