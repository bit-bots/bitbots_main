#pragma once

#include <Eigen/Geometry>
#include <bitbots_head_mover/camera_model.hpp>
#include <bitbots_head_mover/field_coverage_map.hpp>
#include <bitbots_head_mover/head_kinematics.hpp>
#include <bitbots_head_mover/types.hpp>
#include <bitbots_head_mover/world_model.hpp>
#include <optional>
#include <vector>

/// Scoring of candidate head targets.
namespace bitbots_head_mover {

/// Relative importance of the individual scoring terms.
///
/// The reward terms are each normalized to [0, 1] on their own, so their weights
/// are directly comparable. The smoothness weight is different in kind: it is a
/// cost per radian subtracted from the total, not a normalized reward, so it is
/// expressed in the same units as the reward weights but multiplies a distance.
struct ScoringWeights {
  /// Keeping the filtered ball estimate in view. Weighted highest because losing
  /// the ball is the most expensive thing the head can do.
  double filtered_ball = 4.0;
  /// Keeping the raw ball detections in view. These are noisier but far more
  /// immediate than the filtered estimate.
  double raw_balls = 2.0;
  /// Keeping a ball reported by a teammate in view, which is what lets the robot
  /// pick up a ball it has not seen itself.
  double team_ball = 1.0;
  /// Looking at parts of the field that have not been observed in a while.
  ///
  /// There is deliberately no separate penalty for looking off the field:
  /// off-field cells carry no interest, so aiming at them simply earns no
  /// coverage. Looking away from the field is punished by what it forgoes,
  /// which also covers aiming at the sky, where nothing projects at all.
  double field_coverage = 1.5;
  /// Keeping detected robots in view, so the obstacle information stays fresh.
  double robots = 0.5;
  /// Cost per radian of joint space distance between a target and the previously
  /// selected one. This is subtracted from the score, so a larger value makes the
  /// head prefer targets close to the last one and keeps it from jumping between
  /// equally attractive parts of the field every cycle.
  double smoothness = 1.5;
};

/// The individual contributions to a candidate's score.
///
/// Kept separate from the total so the terms can be inspected and plotted while
/// tuning the weights, instead of only seeing the number they add up to.
struct ScoreBreakdown {
  double filtered_ball = 0.0;
  double raw_balls = 0.0;
  double team_ball = 0.0;
  double field_coverage = 0.0;
  double robots = 0.0;
  /// Joint space distance, in radians, to the previously selected target. Zero
  /// when there is no previous target. Enters the total as a cost, weighted by
  /// ScoringWeights::smoothness.
  double smoothness_cost = 0.0;
  /// The weighted sum of the reward terms minus the weighted smoothness cost.
  double total = 0.0;
  /// Whether the candidate could be scored at all.
  ///
  /// A candidate that could not be scored must never be compared against one
  /// that could: a zeroed breakdown is indistinguishable from a genuinely
  /// worthless candidate, and the planner would happily select it.
  bool valid = false;
};

/// Everything about the world that a candidate is scored against.
///
/// Bundled into one struct so the scoring interface does not grow a parameter
/// for every input, and so a test can build a world without a running node.
struct ScoringContext {
  /// The robot's pose on the field, i.e. the map to root link transform. The
  /// root link is whichever link the kinematics resolve the camera against.
  Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
  /// The previously selected target. Empty if there is no previous selection,
  /// which disables the smoothness cost.
  std::optional<HeadPosition> previous_target;
};

/// Scores candidate head targets against the current world state.
///
/// A candidate is a single head position and is evaluated as the one camera pose
/// it points at, rather than as a motion over time: the head is driven towards
/// the selected target by a separate controller, so the planner only has to
/// decide where to look, not how to get there.
class ActiveVisionScorer {
 public:
  /// The robot pose is required rather than defaulted: the coverage distance
  /// falloff is resolved against it, and a defaulted pose would quietly weight
  /// every cell as if the robot stood on the center spot.
  ActiveVisionScorer(const HeadKinematics& kinematics, const CameraModel& camera, const WorldModel& world,
                     const FieldCoverageMap& coverage, const Eigen::Isometry3d& robot_pose);

  void setWeights(const ScoringWeights& weights) { weights_ = weights; }
  const ScoringWeights& weights() const { return weights_; }

  void setVisibilityWeighting(const VisibilityWeighting& weighting) { visibility_ = weighting; }

  /// Distance at which a field cell counts half as much for coverage.
  ///
  /// A patch of ground twice as far away subtends roughly a quarter of the image,
  /// so a view aimed at the far half of the field sweeps up far more cells than
  /// a close one for the same effort. Without a falloff the coverage term would
  /// therefore reward distant overview poses, even though a ball at that range
  /// is barely detectable. The weighting is quadratic in the distance, which is
  /// what cancels that growth, so the term measures how much of the image is
  /// usefully spent rather than how much ground area it happens to cover.
  void setCoverageDistanceHalfWeight(double distance) { coverage_distance_half_weight_ = distance; }

  /// Collect the parts of the world state that every candidate is scored
  /// against and that do not change while a batch is scored.
  ///
  /// Recomputing these per candidate would repeat the same grid sum and the same
  /// allocations for every one of them, which measurably dominated the scoring.
  /// The constructor already does this, so it only has to be called again when
  /// the world or the coverage map changed after the scorer was built.
  ///
  /// The distance falloff is resolved against the robot's position rather than
  /// against each candidate's camera pose. The head only shifts the camera by
  /// centimeters, and holding it fixed keeps the coverage denominator identical
  /// for every candidate, which is what makes their scores comparable.
  void prepare(const Eigen::Isometry3d& robot_pose);

  /// Score a single candidate target.
  ///
  /// The result carries a valid flag; an invalid result means the candidate
  /// could not be scored and must be discarded rather than compared.
  ScoreBreakdown score(const HeadPosition& target, const ScoringContext& context) const;

  /// The camera pose in the map frame for a given head configuration.
  ///
  /// Returns nullopt if the kinematics could not resolve the pose.
  std::optional<Eigen::Isometry3d> cameraPoseInMap(const HeadPosition& position,
                                                   const Eigen::Isometry3d& robot_pose) const;

  /// Record what a head position actually observed into the coverage map.
  ///
  /// This is the counterpart to the coverage scoring: the score asks what a
  /// candidate would see, this records what the executed motion did see.
  /// Returns false if nothing could be recorded, so the caller can report it
  /// instead of silently continuing with a coverage map that stopped updating.
  bool recordObservation(FieldCoverageMap& coverage, const HeadPosition& position,
                         const Eigen::Isometry3d& robot_pose) const;

 private:
  const HeadKinematics& kinematics_;
  const CameraModel& camera_;
  const WorldModel& world_;
  const FieldCoverageMap& coverage_;
  ScoringWeights weights_;
  VisibilityWeighting visibility_;

  double coverage_distance_half_weight_ = 3.0;

  // Per cycle invariants, filled by prepare()
  /// Distance falloff of every cell, resolved against the robot's position.
  std::vector<double> cell_distance_weights_;
  /// The distance weighted interest the field currently holds, which is what a
  /// candidate's covered interest is expressed as a fraction of.
  double available_interest_ = 0.0;
  std::vector<TimedTarget> team_balls_;
  std::vector<TimedTarget> filtered_ball_;
};

/// The visibility of a set of targets from one camera pose, in [0, 1].
///
/// The result is how well the targets are framed, multiplied by how much the
/// best of them is trusted. Framing is a weighted average rather than a maximum,
/// so a view covering several agreeing detections beats one that catches a
/// single outlier; the trust factor is applied separately because an average
/// would normalize the weights away and rate one uncertain target exactly like a
/// certain one. Returns zero when there is nothing to look at.
///
/// Takes the map to camera transform rather than the camera pose, because the
/// caller evaluates several target sets against the same pose and inverting it
/// once per set was pure overhead.
double targetVisibility(const std::vector<TimedTarget>& targets, const Eigen::Isometry3d& map_to_camera,
                        const CameraModel& camera, const VisibilityWeighting& weighting);

}  // namespace bitbots_head_mover
