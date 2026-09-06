#include <algorithm>
#include <bitbots_head_mover/field_coverage_map.hpp>
#include <cmath>

namespace bitbots_head_mover {

FieldCoverageMap::FieldCoverageMap(const FieldCoverageConfig& config) : config_(config) { build(); }

void FieldCoverageMap::build() {
  centers_.clear();
  observations_.clear();
  out_of_field_.clear();

  // A non positive cell size would produce an unbounded grid
  if (!(config_.cell_size > 0.0)) {
    cells_x_ = 0;
    cells_y_ = 0;
    return;
  }

  const double extent_x = config_.field_length + 2.0 * config_.margin;
  const double extent_y = config_.field_width + 2.0 * config_.margin;

  cells_x_ = static_cast<size_t>(std::max(1.0, std::ceil(extent_x / config_.cell_size)));
  cells_y_ = static_cast<size_t>(std::max(1.0, std::ceil(extent_y / config_.cell_size)));

  centers_.reserve(cells_x_ * cells_y_);
  observations_.assign(cells_x_ * cells_y_, 0.0);
  out_of_field_.reserve(cells_x_ * cells_y_);

  // The map frame has its origin in the center of the field, so the grid is
  // centered on the origin as well. The origin has to follow the rounded up cell
  // count rather than the requested extent: taking it from the requested extent
  // would push the grid off center by the rounding remainder whenever the extent
  // is not a multiple of the cell size, and would also disagree with the origin
  // the debug occupancy grid reports.
  const double half_length = config_.field_length / 2.0;
  const double half_width = config_.field_width / 2.0;
  const double origin_x = -(static_cast<double>(cells_x_) * config_.cell_size) / 2.0;
  const double origin_y = -(static_cast<double>(cells_y_) * config_.cell_size) / 2.0;

  for (size_t iy = 0; iy < cells_y_; iy++) {
    for (size_t ix = 0; ix < cells_x_; ix++) {
      const double x = origin_x + (static_cast<double>(ix) + 0.5) * config_.cell_size;
      const double y = origin_y + (static_cast<double>(iy) + 0.5) * config_.cell_size;
      centers_.emplace_back(x, y, 0.0);
      out_of_field_.push_back(std::abs(x) > half_length || std::abs(y) > half_width);
    }
  }
}

double FieldCoverageMap::interest(size_t index) const {
  assert(index < observations_.size() && "coverage cell index out of range");
  // Looking off the field never gains us anything, so those cells carry no
  // interest at all. They are still part of the grid because the scoring wants
  // to know when a candidate is aimed at them.
  if (out_of_field_[index]) {
    return 0.0;
  }
  return 1.0 - observations_[index];
}

void FieldCoverageMap::decay(double dt) {
  if (dt <= 0.0 || !(config_.half_life > 0.0)) {
    return;
  }

  // Exponential decay, so the configured half life is the time after which an
  // observation counts for half as much
  const double factor = std::exp(-dt * M_LN2 / config_.half_life);
  for (double& observation : observations_) {
    observation *= factor;
  }
}

void FieldCoverageMap::observe(size_t index, double quality) {
  assert(index < observations_.size() && "coverage cell index out of range");
  // Seeing a cell badly must not erase the memory of having seen it well
  observations_[index] = std::max(observations_[index], std::clamp(quality, 0.0, 1.0));
}

void FieldCoverageMap::reset() { std::fill(observations_.begin(), observations_.end(), 0.0); }

double FieldCoverageMap::totalInterest() const {
  double total = 0.0;
  for (size_t index = 0; index < size(); index++) {
    total += interest(index);
  }
  return total;
}

}  // namespace bitbots_head_mover
