#include <algorithm>
#include <bitbots_head_mover/active_vision.hpp>
#include <cmath>

namespace bitbots_head_mover {

ActiveVision::ActiveVision() : sampler_(SamplerConfig{}) {}

const char* describe(ActiveVisionFailure failure) {
  switch (failure) {
    case ActiveVisionFailure::None:
      return "no failure";
    case ActiveVisionFailure::NotReady:
      return "an input the planner needs has not arrived yet";
    case ActiveVisionFailure::NoFeasibleCandidate:
      return "no sampled head trajectory respects the joint and dynamic limits";
    case ActiveVisionFailure::KinematicsFailed:
      return "the head chain could not resolve a camera pose for any candidate";
    case ActiveVisionFailure::InvalidSamplerConfig:
      return "the sampling horizon and midpoint time cannot describe a trajectory";
  }
  return "unknown failure";
}

const char* describe(ActiveVisionReadiness readiness) {
  switch (readiness) {
    case ActiveVisionReadiness::Ready:
      return "ready";
    case ActiveVisionReadiness::MissingRobotDescription:
      return "no usable robot description was received on /robot_description";
    case ActiveVisionReadiness::MissingCameraInfo:
      return "no usable camera info was received";
    case ActiveVisionReadiness::MissingFieldDimensions:
      return "the field dimensions are unknown, so there is no coverage map";
  }
  return "unknown state";
}

bool ActiveVision::setRobotDescription(const std::string& urdf, const HeadChainConfig& chain_config) {
  auto kinematics = HeadKinematics::fromUrdf(urdf, chain_config);
  if (!kinematics) {
    return false;
  }
  // Carry an already known calibration over to the new chain, so a robot
  // description arriving after the calibration does not silently drop it
  if (has_calibration_) {
    kinematics->setCameraCalibration(calibration_);
  }
  kinematics_ = std::move(kinematics);
  return true;
}

bool ActiveVision::setCameraInfo(const sensor_msgs::msg::CameraInfo& info) { return camera_.update(info); }

void ActiveVision::setCameraCalibration(const Eigen::Isometry3d& calibration) {
  calibration_ = calibration;
  has_calibration_ = true;
  if (kinematics_) {
    kinematics_->setCameraCalibration(calibration);
  }
}

void ActiveVision::setFieldCoverageConfig(const FieldCoverageConfig& config) {
  coverage_ = std::make_unique<FieldCoverageMap>(config);
}

ActiveVisionReadiness ActiveVision::readiness() const {
  if (!kinematics_) {
    return ActiveVisionReadiness::MissingRobotDescription;
  }
  if (!camera_.valid()) {
    return ActiveVisionReadiness::MissingCameraInfo;
  }
  if (!coverage_ || coverage_->size() == 0) {
    return ActiveVisionReadiness::MissingFieldDimensions;
  }
  return ActiveVisionReadiness::Ready;
}

std::vector<double> ActiveVision::evaluationTimes() const {
  const SamplerConfig& config = sampler_.config();
  std::vector<double> times;
  const int count = std::max(config.evaluation_points, 1);
  times.reserve(static_cast<size_t>(count));
  for (int i = 1; i <= count; i++) {
    // Spread over the horizon, ending at the endpoint. The start is deliberately
    // not evaluated: every candidate begins at the measured head position, so
    // that point scores identically for all of them and can only waste time.
    // With two points this evaluates the midpoint and the goal point.
    times.push_back(config.horizon * static_cast<double>(i) / static_cast<double>(count));
  }
  return times;
}

ActiveVisionResult ActiveVision::plan(const ActiveVisionInput& input) {
  ActiveVisionResult result;
  if (!ready()) {
    result.failure = ActiveVisionFailure::NotReady;
    return result;
  }

  // Catch a sampling configuration that cannot describe a trajectory here, so it
  // is reported as such instead of surfacing as "every candidate was rejected"
  const SamplerConfig& sampler_config = sampler_.config();
  if (!(sampler_config.horizon > 0.0) || !(sampler_config.midpoint_time > 0.0) ||
      sampler_config.midpoint_time >= sampler_config.horizon) {
    result.failure = ActiveVisionFailure::InvalidSamplerConfig;
    return result;
  }

  // Age out detections that are no longer trustworthy
  world_.prune(input.now);

  // Let the record of what was already seen fade, so parts of the field become
  // worth revisiting
  if (has_planned_) {
    coverage_->decay(std::max(0.0, input.now - last_plan_time_));
  }

  ActiveVisionScorer scorer(*kinematics_, camera_, world_, *coverage_, input.robot_pose);
  scorer.setWeights(weights_);
  scorer.setVisibilityWeighting(visibility_);
  scorer.setCoverageDistanceHalfWeight(coverage_distance_half_weight_);

  // Record what the head is looking at right now. This has to happen before
  // scoring, so a candidate does not get rewarded for covering ground that the
  // current head position already covers.
  // A failure here means the kinematics are broken, which the scoring below
  // reports in more detail, so it is not turned into its own failure
  (void)scorer.recordObservation(*coverage_, input.head_position, input.robot_pose);

  // Recording the observation changed the coverage map, so refresh what the
  // batch is scored against
  scorer.prepare(input.robot_pose);

  ScoringContext context;
  context.robot_pose = input.robot_pose;
  context.evaluation_times = evaluationTimes();

  // Measure commitment against where the previous selection would be at the very
  // same moments in time, not against its raw parameterization, because that
  // trajectory started one cycle earlier
  if (has_previous_) {
    const double offset = input.now - previous_plan_time_;
    context.previous_positions.reserve(context.evaluation_times.size());
    for (double time : context.evaluation_times) {
      context.previous_positions.push_back(
          previous_trajectory_.position(std::min(offset + time, previous_trajectory_.duration())));
    }
  }

  result.candidates = sampler_.sample(input.head_position, input.head_velocity, limits_, dynamics_);
  if (result.candidates.empty()) {
    result.failure = ActiveVisionFailure::NoFeasibleCandidate;
    return result;
  }

  result.scores.reserve(result.candidates.size());
  double best_score = -std::numeric_limits<double>::infinity();
  bool have_selection = false;
  for (size_t index = 0; index < result.candidates.size(); index++) {
    ScoreBreakdown breakdown = scorer.score(result.candidates[index].trajectory, context);
    // A candidate that could not be scored is discarded rather than compared:
    // its zeroed total would look like a merely unattractive candidate and could
    // still win if every real candidate scores negative
    if (breakdown.valid && (!have_selection || breakdown.total > best_score)) {
      best_score = breakdown.total;
      result.selected = index;
      have_selection = true;
    }
    if (!breakdown.valid) {
      result.unscorable_candidates++;
    }
    result.scores.push_back(breakdown);
  }

  if (!have_selection) {
    result.failure = ActiveVisionFailure::KinematicsFailed;
    return result;
  }

  const HeadTrajectory& selected = result.candidates[result.selected].trajectory;

  // Command a point a little way along the selected trajectory rather than its
  // start. The trajectory starts at the measured head position, so commanding
  // its start would ask the head to stay exactly where it already is and it
  // would never move. Only this leading segment is ever executed before the next
  // cycle replans, which is what lets the head react immediately while the
  // commitment term keeps it from flickering.
  const double lookahead = std::clamp(command_lookahead_, 0.0, selected.duration());
  result.position = limits_.clamp(selected.position(lookahead));
  result.velocity = selected.velocity(lookahead);
  result.valid = true;

  previous_trajectory_ = selected;
  previous_plan_time_ = input.now;
  has_previous_ = true;
  last_plan_time_ = input.now;
  has_planned_ = true;

  return result;
}

void ActiveVision::reset() {
  world_.clear();
  if (coverage_) {
    coverage_->reset();
  }
  has_previous_ = false;
  has_planned_ = false;
}

}  // namespace bitbots_head_mover
