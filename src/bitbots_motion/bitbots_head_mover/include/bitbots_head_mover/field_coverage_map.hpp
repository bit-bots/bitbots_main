#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

/// A coarse record of which parts of the field were looked at recently.
namespace bitbots_head_mover {

/// Extent and behavior of the field coverage map.
struct FieldCoverageConfig {
  /// Field dimensions, as reported by the parameter blackboard.
  double field_length = 9.0;
  double field_width = 6.0;
  /// How far beyond the field lines the map extends. The area outside the field
  /// still has to be represented, because looking at it is what we want to
  /// discourage, and that requires knowing it is there.
  double margin = 1.0;
  /// Edge length of a grid cell. This is deliberately coarse: the map only has
  /// to steer the head, not localize anything, and every cell is projected into
  /// the camera for every evaluated point of every candidate.
  double cell_size = 1.0;
  /// Time after which an observation has faded to half its value.
  double half_life = 8.0;
};

/// Tracks how recently each part of the field was seen.
///
/// Cells that have not been looked at for a while become interesting again,
/// which is what makes the head sweep the field on its own instead of staring at
/// whatever it already knows about. The map lives in the map frame, so the
/// record survives the robot walking around.
class FieldCoverageMap {
 public:
  explicit FieldCoverageMap(const FieldCoverageConfig& config = {});

  const FieldCoverageConfig& config() const { return config_; }

  /// Number of cells in the grid.
  size_t size() const { return centers_.size(); }
  size_t cellsX() const { return cells_x_; }
  size_t cellsY() const { return cells_y_; }

  /// The cell centers in the map frame, in row major order.
  ///
  /// The z coordinate is zero, i.e. the cells sit on the ground plane.
  const std::vector<Eigen::Vector3d>& cellCenters() const { return centers_; }

  /// How much attention a cell currently deserves, in [0, 1].
  ///
  /// One means it has not been looked at in a long time. Cells outside the field
  /// are never interesting, so looking at them can only be a waste.
  double interest(size_t index) const;

  /// How recently a cell was observed, in [0, 1], where one is just now.
  double observation(size_t index) const { return observations_[index]; }

  /// Whether a cell lies outside the field lines.
  bool isOutOfField(size_t index) const { return out_of_field_[index]; }

  /// Let all observations fade towards being unobserved.
  void decay(double dt);

  /// Record that a cell was observed, with a quality in [0, 1].
  ///
  /// Observing a cell that was already observed does not reduce its record.
  void observe(size_t index, double quality);

  /// Forget every observation, e.g. after the localization was reset.
  void reset();

  /// The sum of the interest of all cells, used to normalize coverage scores.
  double totalInterest() const;

 private:
  /// Build the grid from the configured extents.
  void build();

  FieldCoverageConfig config_;
  size_t cells_x_ = 0;
  size_t cells_y_ = 0;
  std::vector<Eigen::Vector3d> centers_;
  std::vector<double> observations_;
  std::vector<bool> out_of_field_;
};

}  // namespace bitbots_head_mover
