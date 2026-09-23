#include <algorithm>
#include <bitbots_head_mover/active_vision.hpp>
#include <cmath>
#include <limits>

namespace bitbots_head_mover {

ActiveVision::ActiveVision() : sampler_(SamplerConfig{}) {}

const char* describe(ActiveVisionFailure failure) {
  switch (failure) {
    case ActiveVisionFailure::None:
      return "no failure";
    case ActiveVisionFailure::NotReady:
      return "an input the planner needs has not arrived yet";
    case ActiveVisionFailure::KinematicsFailed:
      return "the head chain could not resolve a camera pose for any candidate";
    case ActiveVisionFailure::InvalidSamplerConfig:
      return "the sampler cannot draw any candidate with its current configuration";
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

ActiveVisionResult ActiveVision::plan(const ActiveVisionInput& input) {
  ActiveVisionResult result;
  if (!ready()) {
    result.failure = ActiveVisionFailure::NotReady;
    return result;
  }

  // A sampler that cannot draw is reported as such instead of surfacing as
  // "every candidate was rejected"
  const SamplerConfig& sampler_config = sampler_.config();
  const double weight_sum = std::max(0.0, sampler_config.last_target_weight) +
                            std::max(0.0, sampler_config.current_position_weight) +
                            std::max(0.0, sampler_config.uniform_weight);
  if (sampler_config.sample_count < 0 || !(weight_sum > 0.0)) {
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
  context.previous_target = previous_target_;

  // The current position and the previous target seed the sampling distribution,
  // so the search stays concentrated where the head is and where it was heading
  result.candidates = sampler_.sample(input.head_position, previous_target_, limits_);
  if (result.candidates.empty()) {
    // The sampler always offers the current position unless it is misconfigured,
    // which the check above already ruled out; guard anyway
    result.failure = ActiveVisionFailure::InvalidSamplerConfig;
    return result;
  }

  result.scores.reserve(result.candidates.size());
  double best_score = -std::numeric_limits<double>::infinity();
  bool have_selection = false;
  for (size_t index = 0; index < result.candidates.size(); index++) {
    ScoreBreakdown breakdown = scorer.score(result.candidates[index].target, context);
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

  result.target = limits_.clamp(result.candidates[result.selected].target);

  // Rate limited controller: step the setpoint straight towards the target in
  // joint space, at the maximum speed until the head comes within the approach
  // distance, then ramp the speed down linearly so it glides to a stop on the
  // target instead of snapping to it. This is what replaces the planned
  // trajectory; the smoothness cost keeps the target itself from jumping between
  // cycles.
  const HeadPosition& current = input.head_position;
  const double error_yaw = result.target.yaw - current.yaw;
  const double error_pitch = result.target.pitch - current.pitch;
  const double distance = std::hypot(error_yaw, error_pitch);

  // Below this the head is on the target; moving would only chase sampling noise.
  constexpr double kAtTargetEpsilon = 1e-6;
  if (distance < kAtTargetEpsilon) {
    result.velocity = {0.0, 0.0};
    result.position = result.target;
  } else {
    // Unit direction towards the target, so the head travels in a straight line.
    const double dir_yaw = error_yaw / distance;
    const double dir_pitch = error_pitch / distance;

    // Full speed along this direction that still respects both per-axis speed
    // caps: scale the unit direction up until the first axis hits its own limit.
    const double speed_cap = std::min(controller_.max_velocity.yaw / std::max(std::abs(dir_yaw), kAtTargetEpsilon),
                                      controller_.max_velocity.pitch / std::max(std::abs(dir_pitch), kAtTargetEpsilon));

    // Travel at full speed until within the approach distance, then ramp down
    // linearly to zero as the remaining distance shrinks.
    const double ramp = std::clamp(distance / std::max(controller_.approach_distance, kAtTargetEpsilon), 0.0, 1.0);
    const double speed = speed_cap * ramp;

    // Advance the setpoint by one control period, never stepping past the target.
    const double step = std::min(speed * controller_.control_period, distance);
    result.velocity = {dir_yaw * speed, dir_pitch * speed};
    result.position = limits_.clamp({current.yaw + dir_yaw * step, current.pitch + dir_pitch * step});
  }
  result.valid = true;

  previous_target_ = result.target;
  last_plan_time_ = input.now;
  has_planned_ = true;

  return result;
}

void ActiveVision::reset() {
  world_.clear();
  if (coverage_) {
    coverage_->reset();
  }
  previous_target_.reset();
  has_planned_ = false;
}

}  // namespace bitbots_head_mover
