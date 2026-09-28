#include <algorithm>
#include <bitbots_head_mover/active_vision_scorer.hpp>
#include <cmath>

namespace bitbots_head_mover {

double targetVisibility(const std::vector<TimedTarget>& targets, const Eigen::Isometry3d& map_to_camera,
                        const CameraModel& camera, const VisibilityWeighting& weighting) {
  if (targets.empty()) {
    return 0.0;
  }

  double weighted_sum = 0.0;
  double total_weight = 0.0;
  double best_weight = 0.0;
  for (const auto& target : targets) {
    const double weight = std::max(target.weight, 0.0);
    weighted_sum += weight * camera.visibility(map_to_camera * target.position, weighting);
    total_weight += weight;
    best_weight = std::max(best_weight, weight);
  }

  if (!(total_weight > 0.0)) {
    return 0.0;
  }

  // How well the targets are framed, letting the more trustworthy ones dominate
  const double framing = weighted_sum / total_weight;
  // How much there is to be trusted in the first place. This has to be a
  // separate factor: the weighted average alone normalizes the weights away, so
  // a single very uncertain target would score exactly like a certain one.
  return framing * best_weight;
}

ActiveVisionScorer::ActiveVisionScorer(const HeadKinematics& kinematics, const CameraModel& camera,
                                       const WorldModel& world, const FieldCoverageMap& coverage,
                                       const Eigen::Isometry3d& robot_pose)
    : kinematics_(kinematics), camera_(camera), world_(world), coverage_(coverage) {
  // Prepare right away so a scorer is never in a state where scoring silently
  // returns zero because the caller forgot to prime it
  prepare(robot_pose);
}

std::optional<Eigen::Isometry3d> ActiveVisionScorer::cameraPoseInMap(const HeadPosition& position,
                                                                     const Eigen::Isometry3d& robot_pose) const {
  // The kinematics resolve the camera against the robot's root link, the robot
  // pose puts that root link onto the field
  const auto camera_in_root = kinematics_.cameraPose(position);
  if (!camera_in_root) {
    return std::nullopt;
  }
  return robot_pose * *camera_in_root;
}

bool ActiveVisionScorer::recordObservation(FieldCoverageMap& coverage, const HeadPosition& position,
                                           const Eigen::Isometry3d& robot_pose) const {
  if (!camera_.valid()) {
    return false;
  }

  const auto camera_pose = cameraPoseInMap(position, robot_pose);
  if (!camera_pose) {
    return false;
  }

  const Eigen::Isometry3d map_to_camera = camera_pose->inverse();
  const auto& centers = coverage.cellCenters();
  for (size_t index = 0; index < centers.size(); index++) {
    const double quality = camera_.visibility(map_to_camera * centers[index], visibility_);
    if (quality > 0.0) {
      coverage.observe(index, quality);
    }
  }
  return true;
}

void ActiveVisionScorer::prepare(const Eigen::Isometry3d& robot_pose) {
  team_balls_ = world_.teamBalls();
  filtered_ball_.clear();
  if (world_.filteredBall()) {
    filtered_ball_.push_back(*world_.filteredBall());
  }

  // Resolve the distance falloff of every cell against where the robot stands.
  // A cell twice as far away covers about a quarter of the image, so weighting
  // by the inverse square of the distance cancels the head start that distant
  // cells would otherwise have from sheer count.
  const auto& centers = coverage_.cellCenters();
  const Eigen::Vector3d robot_position = robot_pose.translation();
  const double half_weight = coverage_distance_half_weight_ > 0.0 ? coverage_distance_half_weight_ : 1.0;

  cell_distance_weights_.resize(centers.size());
  available_interest_ = 0.0;
  for (size_t index = 0; index < centers.size(); index++) {
    // Ground distance, so the camera's height above the field does not make
    // everything look uniformly far away
    const double dx = centers[index].x() - robot_position.x();
    const double dy = centers[index].y() - robot_position.y();
    const double normalized = std::sqrt(dx * dx + dy * dy) / half_weight;
    cell_distance_weights_[index] = 1.0 / (1.0 + normalized * normalized);
    available_interest_ += coverage_.interest(index) * cell_distance_weights_[index];
  }
}

ScoreBreakdown ActiveVisionScorer::score(const HeadPosition& target, const ScoringContext& context) const {
  ScoreBreakdown breakdown;
  if (!camera_.valid()) {
    return breakdown;
  }

  const auto camera_pose = cameraPoseInMap(target, context.robot_pose);
  if (!camera_pose) {
    // A pose the kinematics could not resolve leaves the candidate unscorable
    // rather than merely unattractive, so it is reported as such
    return ScoreBreakdown{};
  }
  // Only the inverse is ever needed, so the forward pose is never formed
  const Eigen::Isometry3d map_to_camera = camera_pose->inverse();

  breakdown.filtered_ball = targetVisibility(filtered_ball_, map_to_camera, camera_, visibility_);
  breakdown.raw_balls = targetVisibility(world_.rawBalls(), map_to_camera, camera_, visibility_);
  breakdown.team_ball = targetVisibility(team_balls_, map_to_camera, camera_, visibility_);
  breakdown.robots = targetVisibility(world_.robots(), map_to_camera, camera_, visibility_);

  // Walk the coverage grid and accumulate how much outstanding attention this
  // view would satisfy. Cells off the field carry no interest, so aiming at
  // them earns nothing; no separate penalty is needed, and unlike one it also
  // covers aiming at the sky, where no cell projects at all.
  const auto& centers = coverage_.cellCenters();
  double covered_interest = 0.0;
  for (size_t index = 0; index < centers.size(); index++) {
    const double interest = coverage_.interest(index);
    if (interest <= 0.0) {
      continue;
    }
    const double quality = camera_.visibility(map_to_camera * centers[index], visibility_);
    if (quality <= 0.0) {
      continue;
    }
    covered_interest += quality * interest * cell_distance_weights_[index];
  }

  if (available_interest_ > 0.0) {
    // Numerator and denominator carry the same distance weighting, so the term
    // is the fraction of the reachable outstanding attention this view
    // satisfies rather than the raw ground area it happens to cover
    breakdown.field_coverage = std::min(covered_interest / available_interest_, 1.0);
  }

  // The joint space distance to the previous target. Left at zero on the first
  // cycle, where there is nothing to stay close to.
  if (context.previous_target) {
    breakdown.smoothness_cost =
        std::hypot(target.yaw - context.previous_target->yaw, target.pitch - context.previous_target->pitch);
  }

  breakdown.total = weights_.filtered_ball * breakdown.filtered_ball + weights_.raw_balls * breakdown.raw_balls +
                    weights_.team_ball * breakdown.team_ball + weights_.field_coverage * breakdown.field_coverage +
                    weights_.robots * breakdown.robots - weights_.smoothness * breakdown.smoothness_cost;
  breakdown.valid = true;

  return breakdown;
}

}  // namespace bitbots_head_mover
