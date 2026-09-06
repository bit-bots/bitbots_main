#pragma once

#include <Eigen/Geometry>
#include <bitbots_head_mover/types.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <memory>
#include <optional>
#include <string>

/// Forward kinematics of the head chain.
namespace bitbots_head_mover {

/// The links and joints the head chain is built from.
struct HeadChainConfig {
  /// The link the resulting camera pose is expressed in.
  std::string root_link = "base_link";
  /// The optical frame at the end of the chain, as it appears in the robot description.
  std::string tip_link = "camera_optical_frame_left_uncalibrated";
  std::string yaw_joint = "head_yaw_joint";
  std::string pitch_joint = "head_pitch_joint";
};

/// Resolves the camera pose for an arbitrary head configuration.
///
/// Being able to evaluate head positions the robot is not currently in is what
/// makes sampling based head control possible: TF only ever knows where the head
/// actually is. The chain is taken from the robot description, so the geometry
/// stays consistent with the model the rest of the stack uses.
///
/// The class is not copyable because the KDL solver refers to the chain it was
/// constructed from; use the returned handle directly.
class HeadKinematics {
 public:
  HeadKinematics(const HeadKinematics&) = delete;
  HeadKinematics& operator=(const HeadKinematics&) = delete;

  /// Build the head chain from a robot description.
  ///
  /// Returns nullptr if the URDF cannot be parsed, if there is no chain between
  /// the configured links, or if that chain's movable joints are not exactly the
  /// two configured head joints.
  static std::unique_ptr<HeadKinematics> fromUrdf(const std::string& urdf, const HeadChainConfig& config = {});

  /// The camera pose relative to the root link for a given head configuration.
  ///
  /// Includes the camera calibration if one was set. Returns nullopt if the
  /// solver fails, rather than an identity pose: an identity would silently
  /// place the camera at the robot's origin looking along its x axis, which is a
  /// perfectly plausible pose that would be scored as if it were real.
  std::optional<Eigen::Isometry3d> cameraPose(const HeadPosition& position) const;

  /// The head joint limits as declared in the robot description.
  ///
  /// These are the mechanical limits of the joints. The head mover additionally
  /// applies its own, usually tighter, configured limits.
  const HeadLimits& urdfLimits() const { return urdf_limits_; }

  /// Append a fixed transform to every camera pose.
  ///
  /// The extrinsic camera calibration is published as a separate transform
  /// instead of being part of the robot description, so it has to be composed
  /// onto the chain's tip to arrive at the optical frame that the vision
  /// pipeline reports its detections in. The transform does not depend on the
  /// head configuration, so looking it up once is enough.
  void setCameraCalibration(const Eigen::Isometry3d& calibration) {
    calibration_ = calibration;
    has_calibration_ = true;
  }

  /// Whether a camera calibration was set.
  bool hasCameraCalibration() const { return has_calibration_; }

 private:
  HeadKinematics() = default;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_;
  /// Scratch joint vector, reused so that resolving a camera pose does not
  /// allocate. The scoring resolves one per evaluated point of every candidate,
  /// which made this an allocation per call in the control loop. Mutable because
  /// it is an implementation detail of a logically const query; this makes
  /// cameraPose() unsafe to call on one instance from several threads at once,
  /// which the single planning loop never does.
  mutable KDL::JntArray joints_;
  /// Index of the head joints among the chain's movable joints.
  unsigned int yaw_index_ = 0;
  unsigned int pitch_index_ = 0;
  HeadLimits urdf_limits_;
  Eigen::Isometry3d calibration_ = Eigen::Isometry3d::Identity();
  bool has_calibration_ = false;
};

}  // namespace bitbots_head_mover
