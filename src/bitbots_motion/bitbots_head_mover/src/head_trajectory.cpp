#include <algorithm>
#include <bitbots_head_mover/head_trajectory.hpp>
#include <cmath>

namespace bitbots_head_mover {

namespace {
/// Factor to convert an angle in degrees to radians.
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

void HeadTrajectory::addPoint(double time, const HeadPosition& position, const HeadVelocity& velocity,
                              const HeadAcceleration& acceleration) {
  yaw_.addPoint(time, position.yaw, velocity.yaw, acceleration.yaw);
  pitch_.addPoint(time, position.pitch, velocity.pitch, acceleration.pitch);
  duration_ = time;
  size_++;
}

void HeadTrajectory::finalize() {
  // A single waypoint does not describe a movement, so there is nothing to interpolate
  if (size_ < 2) {
    valid_ = false;
    return;
  }
  yaw_.computeSplines();
  pitch_.computeSplines();
  valid_ = true;
}

HeadPosition HeadTrajectory::position(double time) const {
  if (!valid_) {
    return {};
  }
  return {yaw_.pos(time), pitch_.pos(time)};
}

HeadVelocity HeadTrajectory::velocity(double time) const {
  if (!valid_) {
    return {};
  }
  // Signed velocity, so callers can tell the direction of travel. Motor goals
  // take the absolute value because they carry a speed rather than a velocity.
  return {yaw_.vel(time), pitch_.vel(time)};
}

HeadAcceleration HeadTrajectory::acceleration(double time) const {
  if (!valid_) {
    return {};
  }
  return {yaw_.acc(time), pitch_.acc(time)};
}

double SearchPatternTrajectory::phase(double elapsed) const {
  if (!valid()) {
    return 0.0;
  }
  // Play the transition from the previous head position once, then loop only
  // over the cyclic part of the trajectory
  if (elapsed <= transition_duration) {
    return elapsed;
  }
  return transition_duration + std::fmod(elapsed - transition_duration, cycle_duration);
}

SearchPatternTrajectory buildSearchPatternTrajectory(const std::vector<HeadPosition>& pattern_deg, double cycle_time,
                                                     const HeadPosition& start, double transition_speed) {
  SearchPatternTrajectory result;

  if (pattern_deg.empty() || cycle_time <= 0.0 || transition_speed <= 0.0) {
    return result;
  }

  // Convert all keypoints to radians and close the loop
  std::vector<HeadPosition> pts_rad;
  pts_rad.reserve(pattern_deg.size() + 1);
  for (const auto& keyframe : pattern_deg) {
    pts_rad.push_back({keyframe.yaw * kDegToRad, keyframe.pitch * kDegToRad});
  }
  pts_rad.push_back(pts_rad[0]);

  // Compute segment arc lengths and total
  std::vector<double> lengths;
  lengths.reserve(pts_rad.size() - 1);
  double total_length = 0.0;
  for (size_t i = 1; i < pts_rad.size(); i++) {
    double dyaw = pts_rad[i].yaw - pts_rad[i - 1].yaw;
    double dpitch = pts_rad[i].pitch - pts_rad[i - 1].pitch;
    double len = std::sqrt(dyaw * dyaw + dpitch * dpitch);
    lengths.push_back(len);
    total_length += len;
  }

  double t = 0.0;

  // Prepend a transition segment from the current head position to the first waypoint
  // of the pattern, so we don't jump there abruptly when the head mode changes.
  // Its duration is based on the distance and the configured transition speed.
  double transition_distance =
      std::sqrt(std::pow(pts_rad[0].yaw - start.yaw, 2) + std::pow(pts_rad[0].pitch - start.pitch, 2));
  result.transition_duration = transition_distance / transition_speed;
  if (result.transition_duration > 0.0) {
    result.trajectory.addPoint(t, start);
    t = result.transition_duration;
  }

  // Add waypoints with timestamps proportional to arc length
  result.trajectory.addPoint(t, pts_rad[0]);
  for (size_t i = 0; i < lengths.size(); i++) {
    double fraction = (total_length > 0.0) ? lengths[i] / total_length : 1.0 / static_cast<double>(lengths.size());
    t += fraction * cycle_time;
    result.trajectory.addPoint(t, pts_rad[i + 1]);
  }

  result.cycle_duration = cycle_time;
  result.trajectory.finalize();
  return result;
}

double calculateLowerSpeed(double delta_faster_joint, double delta_joint, double speed) {
  double estimated_time = delta_faster_joint / speed;
  if (estimated_time != 0) {
    return delta_joint / estimated_time;
  } else {
    return 0;
  }
}

HeadVelocity adjustSpeeds(const HeadPosition& goal, const HeadPosition& current, const HeadVelocity& max_speeds) {
  // Calculate the delta between the current and the goal positions
  double delta_yaw = std::abs(goal.yaw - current.yaw);
  double delta_pitch = std::abs(goal.pitch - current.pitch);

  HeadVelocity speeds = max_speeds;
  // Check which axis has to move further and adjust the speed of the other axis so both reach the goal at the same
  // time
  if (delta_yaw > delta_pitch) {
    // Slow down the pitch axis to match the time it takes for the yaw axis to reach the goal
    speeds.pitch = std::min(max_speeds.pitch, calculateLowerSpeed(delta_yaw, delta_pitch, max_speeds.yaw));
  } else {
    // Slow down the yaw axis to match the time it takes for the pitch axis to reach the goal
    speeds.yaw = std::min(max_speeds.yaw, calculateLowerSpeed(delta_pitch, delta_yaw, max_speeds.pitch));
  }
  return speeds;
}

}  // namespace bitbots_head_mover
