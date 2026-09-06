#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

/// The detections the head should pay attention to.
namespace bitbots_head_mover {

/// A detection the head could look at, in the map frame.
struct TimedTarget {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// How much attention this detection deserves, in [0, 1].
  double weight = 1.0;
  /// When the detection was made, in seconds.
  double stamp = 0.0;
};

/// How long detections stay relevant and how their weight is derived.
struct WorldModelConfig {
  /// Filtered estimates are kept longest because the filter already smooths over
  /// individual missed frames.
  double filtered_ball_timeout = 5.0;
  /// Raw detections are only meaningful for a moment, they are what tells the
  /// head that something is right there at this instant.
  double raw_ball_timeout = 0.5;
  double team_ball_timeout = 3.0;
  double robot_timeout = 1.0;
  /// The covariance at which a filtered estimate's weight has dropped to half.
  /// Larger covariances keep lowering the weight without ever reaching zero.
  double covariance_half_weight = 0.5;
};

/// Turn the covariance of an estimate into a weight in (0, 1].
///
/// A perfectly known position keeps its full weight, and the weight halves once
/// the covariance reaches the configured scale. The scale has to be positive;
/// the parameter validation enforces that, and a non positive one is a
/// configuration error rather than a case to be handled.
double weightFromCovariance(double covariance, double covariance_half_weight);

/// Buffers the detections used to score head positions.
///
/// Every position is expected in the map frame, so that stored detections stay
/// valid while the robot walks. Entries are dropped once they age past their
/// category's timeout, which is what makes the head stop chasing stale
/// information without any explicit clearing logic.
class WorldModel {
 public:
  explicit WorldModel(const WorldModelConfig& config = {}) : config_(config) {}

  void setConfig(const WorldModelConfig& config) { config_ = config; }
  const WorldModelConfig& config() const { return config_; }

  /// Replace the filtered ball estimate.
  ///
  /// The covariance is expected to be the larger of the two planar variances, so
  /// that an estimate that is uncertain in any direction is treated as uncertain.
  void setFilteredBall(const Eigen::Vector3d& position, double covariance, double stamp);

  /// Replace the current raw ball detections.
  void setRawBalls(std::vector<TimedTarget> balls);

  /// Replace the ball reported by one teammate.
  ///
  /// Teammates are tracked individually so that a single silent robot does not
  /// keep its last ball alive through the others' messages.
  void setTeamBall(uint8_t robot_id, const Eigen::Vector3d& position, double covariance, double stamp);

  /// Replace the current robot detections.
  void setRobots(std::vector<TimedTarget> robots);

  /// Drop every detection that has aged past its timeout.
  ///
  /// Call this once per control cycle before reading any of the accessors.
  void prune(double now);

  const std::optional<TimedTarget>& filteredBall() const { return filtered_ball_; }
  const std::vector<TimedTarget>& rawBalls() const { return raw_balls_; }
  const std::vector<TimedTarget>& robots() const { return robots_; }

  /// The balls currently reported by teammates.
  std::vector<TimedTarget> teamBalls() const;

  /// Whether any ball information at all is available.
  bool hasAnyBall() const;

  /// Forget everything, e.g. after the localization was reset.
  void clear();

 private:
  WorldModelConfig config_;
  std::optional<TimedTarget> filtered_ball_;
  std::vector<TimedTarget> raw_balls_;
  std::vector<TimedTarget> robots_;
  std::map<uint8_t, TimedTarget> team_balls_;
};

}  // namespace bitbots_head_mover
