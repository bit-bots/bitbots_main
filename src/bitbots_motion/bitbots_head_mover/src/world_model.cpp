#include <algorithm>
#include <bitbots_head_mover/world_model.hpp>
#include <cassert>
#include <cmath>

namespace bitbots_head_mover {

namespace {

/// Remove every target that is older than the given timeout.
void pruneOlderThan(std::vector<TimedTarget>& targets, double now, double timeout) {
  targets.erase(std::remove_if(targets.begin(), targets.end(),
                               [&](const TimedTarget& target) { return now - target.stamp > timeout; }),
                targets.end());
}

}  // namespace

double weightFromCovariance(double covariance, double covariance_half_weight) {
  // A negative covariance is meaningless, and a non positive scale would make
  // the weight jump between the extremes instead of fading
  const double clamped_covariance = std::max(covariance, 0.0);
  // A non positive scale is a configuration error the parameter validation
  // rules out. Assert rather than quietly granting full trust to an estimate
  // whose uncertainty we then never account for.
  assert(covariance_half_weight > 0.0 && "covariance_half_weight must be positive");
  if (!(covariance_half_weight > 0.0)) {
    return 1.0;
  }
  // Reaches one half exactly at the configured covariance and keeps decreasing
  // from there without ever becoming zero, so a very uncertain estimate still
  // beats no estimate at all
  return 1.0 / (1.0 + clamped_covariance / covariance_half_weight);
}

void WorldModel::setFilteredBall(const Eigen::Vector3d& position, double covariance, double stamp) {
  filtered_ball_ = TimedTarget{position, weightFromCovariance(covariance, config_.covariance_half_weight), stamp};
}

void WorldModel::setRawBalls(std::vector<TimedTarget> balls) { raw_balls_ = std::move(balls); }

void WorldModel::setTeamBall(uint8_t robot_id, const Eigen::Vector3d& position, double covariance, double stamp) {
  team_balls_[robot_id] =
      TimedTarget{position, weightFromCovariance(covariance, config_.covariance_half_weight), stamp};
}

void WorldModel::setRobots(std::vector<TimedTarget> robots) { robots_ = std::move(robots); }

void WorldModel::prune(double now) {
  if (filtered_ball_ && now - filtered_ball_->stamp > config_.filtered_ball_timeout) {
    filtered_ball_.reset();
  }

  pruneOlderThan(raw_balls_, now, config_.raw_ball_timeout);
  pruneOlderThan(robots_, now, config_.robot_timeout);

  for (auto it = team_balls_.begin(); it != team_balls_.end();) {
    if (now - it->second.stamp > config_.team_ball_timeout) {
      it = team_balls_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<TimedTarget> WorldModel::teamBalls() const {
  std::vector<TimedTarget> balls;
  balls.reserve(team_balls_.size());
  for (const auto& [robot_id, ball] : team_balls_) {
    balls.push_back(ball);
  }
  return balls;
}

bool WorldModel::hasAnyBall() const {
  return filtered_ball_.has_value() || !raw_balls_.empty() || !team_balls_.empty();
}

void WorldModel::clear() {
  filtered_ball_.reset();
  raw_balls_.clear();
  robots_.clear();
  team_balls_.clear();
}

}  // namespace bitbots_head_mover
