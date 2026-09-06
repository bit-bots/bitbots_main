#pragma once

#include <Eigen/Geometry>
#include <bitbots_head_mover/active_vision_scorer.hpp>
#include <bitbots_head_mover/camera_model.hpp>
#include <bitbots_head_mover/field_coverage_map.hpp>
#include <bitbots_head_mover/head_kinematics.hpp>
#include <bitbots_head_mover/target_sampler.hpp>
#include <bitbots_head_mover/types.hpp>
#include <bitbots_head_mover/world_model.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/// Sampling based head control.
namespace bitbots_head_mover {

/// The speed limit and approach behaviour of the controller that drives the head
/// towards the selected target.
///
/// Each cycle the controller moves the setpoint straight towards the target in
/// joint space. It travels at the maximum speed until it comes within the
/// approach distance of the target, then ramps the speed down linearly so the
/// head glides to a stop on the target instead of snapping to it or overshooting.
struct HeadController {
  /// Joint space distance, in radians, within which the speed is ramped down
  /// linearly from the maximum to zero. Beyond it the head travels at full speed;
  /// a larger value makes the head ease into the target more gently.
  double approach_distance = 0.3;
  /// The time step, in seconds, the setpoint is advanced by each cycle. This is
  /// the control loop period, kept explicit so the step does not depend on
  /// wall-clock jitter between cycles.
  double control_period = 0.05;
  /// The maximum commanded joint speed, in radians per second, per axis. The
  /// straight line motion is scaled so neither axis exceeds its own limit.
  HeadVelocity max_velocity{7.0, 7.0};
};

/// Everything the planner needs that is not part of its own state.
struct ActiveVisionInput {
  /// Where the head currently is.
  HeadPosition head_position;
  /// The robot's pose on the field, i.e. the map to root link transform.
  Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
  /// The current time in seconds.
  double now = 0.0;
};

/// Why the planner is not able to run.
enum class ActiveVisionReadiness {
  Ready,
  MissingRobotDescription,
  MissingCameraInfo,
  MissingFieldDimensions,
};

/// Why a planning cycle produced no command.
///
/// Every failure gets its own value so the node can say what actually went
/// wrong. A single "it did not work" would leave an operator guessing between a
/// missing camera, a broken chain and a misconfigured sampler.
enum class ActiveVisionFailure {
  None,
  /// An input the planner needs has not arrived yet.
  NotReady,
  /// The head chain could not resolve a camera pose, so nothing could be scored.
  KinematicsFailed,
  /// The sampler is configured such that it cannot draw any candidate.
  InvalidSamplerConfig,
};

/// A human readable description of a planning failure.
const char* describe(ActiveVisionFailure failure);

/// A human readable description of a missing input.
const char* describe(ActiveVisionReadiness readiness);

/// What the planner decided, plus what it considered while deciding.
struct ActiveVisionResult {
  /// Whether a command could be produced at all.
  bool valid = false;
  /// Why no command was produced. None when valid.
  ActiveVisionFailure failure = ActiveVisionFailure::None;
  /// How many candidates had to be discarded because they could not be scored.
  /// Non zero on an otherwise valid result means the planner chose from fewer
  /// candidates than it sampled, which is worth reporting.
  size_t unscorable_candidates = 0;
  /// The selected target the head is steered towards.
  HeadPosition target;
  /// The head position to command right now, one control step towards the target.
  HeadPosition position;
  /// The joint speeds to command right now while travelling towards the target.
  HeadVelocity velocity;
  /// Every candidate that was scored, kept for the debug output.
  std::vector<Candidate> candidates;
  /// The score of each candidate, in the same order.
  std::vector<ScoreBreakdown> scores;
  /// Index of the selected candidate within the vectors above.
  size_t selected = 0;
};

/// Plans head motion by sampling candidate targets and scoring them.
///
/// Owns the pieces the scoring is built from and drives one planning cycle per
/// call to plan(). Everything that depends on the ROS graph — the robot
/// description, the camera intrinsics, the detections and the robot pose — is
/// pushed in from the outside, so the planner itself stays testable. The head is
/// steered towards the selected target by a rate limited controller that eases
/// into the target, rather than along a planned trajectory.
class ActiveVision {
 public:
  ActiveVision();

  /// Adopt the robot description and build the head chain from it.
  ///
  /// Returns false if no usable head chain could be extracted, in which case the
  /// planner stays unready rather than falling back to a guessed geometry.
  bool setRobotDescription(const std::string& urdf, const HeadChainConfig& chain_config);

  /// Adopt the camera intrinsics. Returns whether they were usable.
  bool setCameraInfo(const sensor_msgs::msg::CameraInfo& info);

  /// Adopt the extrinsic camera calibration, which is not part of the URDF.
  void setCameraCalibration(const Eigen::Isometry3d& calibration);

  /// Build the coverage map once the field dimensions are known.
  void setFieldCoverageConfig(const FieldCoverageConfig& config);

  void setSamplerConfig(const SamplerConfig& config) { sampler_.setConfig(config); }
  void setHeadLimits(const HeadLimits& limits) { limits_ = limits; }
  void setController(const HeadController& controller) { controller_ = controller; }
  void setScoringWeights(const ScoringWeights& weights) { weights_ = weights; }
  void setVisibilityWeighting(const VisibilityWeighting& weighting) { visibility_ = weighting; }
  void setCoverageDistanceHalfWeight(double distance) { coverage_distance_half_weight_ = distance; }
  void setWorldModelConfig(const WorldModelConfig& config) { world_.setConfig(config); }

  /// Whether the planner has everything it needs.
  ActiveVisionReadiness readiness() const;
  bool ready() const { return readiness() == ActiveVisionReadiness::Ready; }

  /// The detections the planner scores against.
  WorldModel& world() { return world_; }
  const WorldModel& world() const { return world_; }

  /// The record of which parts of the field were looked at.
  ///
  /// Only valid once the planner is ready.
  const FieldCoverageMap& coverage() const { return *coverage_; }

  /// The head chain the camera poses are resolved with.
  ///
  /// Only valid once the planner is ready.
  const HeadKinematics& kinematics() const { return *kinematics_; }

  /// The camera the visibility is judged against.
  const CameraModel& camera() const { return camera_; }

  /// The joint limits candidates are drawn within.
  const HeadLimits& headLimits() const { return limits_; }

  /// The shape and number of the sampled candidates.
  const SamplerConfig& samplerConfig() const { return sampler_.config(); }

  /// Run one planning cycle.
  ///
  /// Ages out stale detections, decays the coverage record, folds in what the
  /// head is currently seeing, then samples and scores candidate targets and
  /// steers the head one control step towards the best one. Returns an invalid
  /// result if the planner is not ready or no candidate survived.
  ActiveVisionResult plan(const ActiveVisionInput& input);

  /// Forget the coverage record and all detections.
  void reset();

 private:
  std::unique_ptr<HeadKinematics> kinematics_;
  CameraModel camera_;
  WorldModel world_;
  std::unique_ptr<FieldCoverageMap> coverage_;
  TargetSampler sampler_;

  HeadLimits limits_{{-1.23, 1.23}, {-1.23, 1.01}};
  HeadController controller_;
  ScoringWeights weights_;
  VisibilityWeighting visibility_;
  double coverage_distance_half_weight_ = 3.0;

  /// The extrinsic camera calibration, kept here as well so it survives the
  /// robot description arriving after it.
  Eigen::Isometry3d calibration_ = Eigen::Isometry3d::Identity();
  bool has_calibration_ = false;

  /// The target selected in the previous cycle, which is what the smoothness
  /// cost is measured against and what the sampler concentrates its search on.
  std::optional<HeadPosition> previous_target_;
  double last_plan_time_ = 0.0;
  bool has_planned_ = false;
};

}  // namespace bitbots_head_mover
