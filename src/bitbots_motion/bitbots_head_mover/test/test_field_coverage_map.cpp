#include <gtest/gtest.h>

#include <bitbots_head_mover/field_coverage_map.hpp>
#include <cmath>

using bitbots_head_mover::FieldCoverageConfig;
using bitbots_head_mover::FieldCoverageMap;

namespace {
FieldCoverageConfig makeConfig() {
  FieldCoverageConfig config;
  config.field_length = 9.0;
  config.field_width = 6.0;
  config.margin = 1.0;
  config.cell_size = 0.5;
  config.half_life = 8.0;
  return config;
}
}  // namespace

// ---------------------------------------------------------------------------
// Grid layout
// ---------------------------------------------------------------------------

TEST(FieldCoverageMap, CoversTheFieldPlusTheMargin) {
  FieldCoverageMap map(makeConfig());
  // 9 + 2 * 1 meters at half a meter per cell
  EXPECT_EQ(map.cellsX(), 22u);
  // 6 + 2 * 1 meters at half a meter per cell
  EXPECT_EQ(map.cellsY(), 16u);
  EXPECT_EQ(map.size(), 22u * 16u);
  EXPECT_EQ(map.cellCenters().size(), map.size());
}

TEST(FieldCoverageMap, IsCenteredOnTheFieldOrigin) {
  FieldCoverageMap map(makeConfig());
  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  for (const auto& center : map.cellCenters()) {
    min_x = std::min(min_x, center.x());
    max_x = std::max(max_x, center.x());
    min_y = std::min(min_y, center.y());
    max_y = std::max(max_y, center.y());
  }
  // The map frame's origin is the center of the field
  EXPECT_NEAR(min_x, -max_x, 1e-9);
  EXPECT_NEAR(min_y, -max_y, 1e-9);
}

TEST(FieldCoverageMap, CellsSitOnTheGroundPlane) {
  FieldCoverageMap map(makeConfig());
  for (const auto& center : map.cellCenters()) {
    EXPECT_DOUBLE_EQ(center.z(), 0.0);
  }
}

TEST(FieldCoverageMap, MarksCellsBeyondTheFieldLines) {
  FieldCoverageMap map(makeConfig());
  size_t out_of_field = 0;
  for (size_t index = 0; index < map.size(); index++) {
    const auto& center = map.cellCenters()[index];
    const bool expected = std::abs(center.x()) > 4.5 || std::abs(center.y()) > 3.0;
    EXPECT_EQ(map.isOutOfField(index), expected) << "cell at " << center.x() << ", " << center.y();
    out_of_field += map.isOutOfField(index) ? 1 : 0;
  }
  // The margin has to actually contain something, otherwise the penalty term
  // would never have anything to fire on
  EXPECT_GT(out_of_field, 0u);
  EXPECT_LT(out_of_field, map.size());
}

TEST(FieldCoverageMap, StaysCenteredWhenTheExtentIsNotAMultipleOfTheCellSize) {
  // 9 + 2 * 1 meters does not divide by 0.75, so the cell count is rounded up.
  // The grid still has to stay centered on the field, otherwise every cell is
  // offset by the rounding remainder and the out of field ring is lopsided.
  FieldCoverageConfig config = makeConfig();
  config.cell_size = 0.75;
  FieldCoverageMap map(config);

  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  for (const auto& center : map.cellCenters()) {
    min_x = std::min(min_x, center.x());
    max_x = std::max(max_x, center.x());
    min_y = std::min(min_y, center.y());
    max_y = std::max(max_y, center.y());
  }
  EXPECT_NEAR(min_x, -max_x, 1e-9);
  EXPECT_NEAR(min_y, -max_y, 1e-9);
}

TEST(FieldCoverageMap, OutOfFieldRingIsSymmetricForAnAwkwardCellSize) {
  FieldCoverageConfig config = makeConfig();
  config.cell_size = 0.75;
  FieldCoverageMap map(config);

  // A cell and its mirror image across the origin must agree on whether they
  // are on the field
  for (size_t index = 0; index < map.size(); index++) {
    const auto& center = map.cellCenters()[index];
    bool found_mirror = false;
    for (size_t other = 0; other < map.size(); other++) {
      const auto& mirror = map.cellCenters()[other];
      if (std::abs(mirror.x() + center.x()) < 1e-9 && std::abs(mirror.y() + center.y()) < 1e-9) {
        EXPECT_EQ(map.isOutOfField(index), map.isOutOfField(other));
        found_mirror = true;
        break;
      }
    }
    EXPECT_TRUE(found_mirror) << "no mirror cell for " << center.x() << ", " << center.y();
  }
}

TEST(FieldCoverageMap, DegenerateCellSizeYieldsAnEmptyGrid) {
  FieldCoverageConfig config = makeConfig();
  config.cell_size = 0.0;
  FieldCoverageMap map(config);
  EXPECT_EQ(map.size(), 0u);
  EXPECT_DOUBLE_EQ(map.totalInterest(), 0.0);
}

// ---------------------------------------------------------------------------
// Interest and observation
// ---------------------------------------------------------------------------

TEST(FieldCoverageMap, UnobservedFieldCellsAreFullyInteresting) {
  FieldCoverageMap map(makeConfig());
  for (size_t index = 0; index < map.size(); index++) {
    if (!map.isOutOfField(index)) {
      EXPECT_DOUBLE_EQ(map.interest(index), 1.0);
    }
  }
}

TEST(FieldCoverageMap, CellsOutsideTheFieldAreNeverInteresting) {
  FieldCoverageMap map(makeConfig());
  for (size_t index = 0; index < map.size(); index++) {
    if (map.isOutOfField(index)) {
      EXPECT_DOUBLE_EQ(map.interest(index), 0.0);
    }
  }
}

TEST(FieldCoverageMap, ObservingACellRemovesItsInterest) {
  FieldCoverageMap map(makeConfig());
  size_t field_cell = 0;
  while (map.isOutOfField(field_cell)) {
    field_cell++;
  }

  map.observe(field_cell, 1.0);
  EXPECT_DOUBLE_EQ(map.interest(field_cell), 0.0);
}

TEST(FieldCoverageMap, PartialObservationRemovesPartOfTheInterest) {
  FieldCoverageMap map(makeConfig());
  size_t field_cell = 0;
  while (map.isOutOfField(field_cell)) {
    field_cell++;
  }

  map.observe(field_cell, 0.25);
  EXPECT_DOUBLE_EQ(map.interest(field_cell), 0.75);
}

TEST(FieldCoverageMap, ASecondWorseLookDoesNotEraseTheBetterOne) {
  FieldCoverageMap map(makeConfig());
  size_t field_cell = 0;
  while (map.isOutOfField(field_cell)) {
    field_cell++;
  }

  map.observe(field_cell, 0.9);
  map.observe(field_cell, 0.2);
  EXPECT_DOUBLE_EQ(map.observation(field_cell), 0.9);
}

TEST(FieldCoverageMap, ObservationQualityIsClamped) {
  FieldCoverageMap map(makeConfig());
  map.observe(0, 5.0);
  EXPECT_DOUBLE_EQ(map.observation(0), 1.0);
}

// ---------------------------------------------------------------------------
// Decay
// ---------------------------------------------------------------------------

TEST(FieldCoverageMap, ObservationsHalveAfterTheHalfLife) {
  FieldCoverageMap map(makeConfig());
  map.observe(0, 1.0);
  map.decay(8.0);
  EXPECT_NEAR(map.observation(0), 0.5, 1e-9);
}

TEST(FieldCoverageMap, DecayMakesObservedCellsInterestingAgain) {
  FieldCoverageMap map(makeConfig());
  size_t field_cell = 0;
  while (map.isOutOfField(field_cell)) {
    field_cell++;
  }

  map.observe(field_cell, 1.0);
  ASSERT_DOUBLE_EQ(map.interest(field_cell), 0.0);

  // This is what makes the head come back to a part of the field it already
  // looked at instead of never revisiting it
  map.decay(8.0);
  EXPECT_NEAR(map.interest(field_cell), 0.5, 1e-9);
}

TEST(FieldCoverageMap, DecayNeverGoesNegative) {
  FieldCoverageMap map(makeConfig());
  map.observe(0, 1.0);
  map.decay(1000.0);
  EXPECT_GE(map.observation(0), 0.0);
}

TEST(FieldCoverageMap, NonPositiveTimeStepChangesNothing) {
  FieldCoverageMap map(makeConfig());
  map.observe(0, 1.0);
  map.decay(0.0);
  map.decay(-1.0);
  EXPECT_DOUBLE_EQ(map.observation(0), 1.0);
}

TEST(FieldCoverageMap, DecayIsIndependentOfTheStepSize) {
  FieldCoverageMap one_step(makeConfig());
  FieldCoverageMap many_steps(makeConfig());
  one_step.observe(0, 1.0);
  many_steps.observe(0, 1.0);

  one_step.decay(4.0);
  for (int i = 0; i < 80; i++) {
    many_steps.decay(0.05);
  }
  EXPECT_NEAR(one_step.observation(0), many_steps.observation(0), 1e-9);
}

// ---------------------------------------------------------------------------
// Aggregate
// ---------------------------------------------------------------------------

TEST(FieldCoverageMap, TotalInterestCountsOnlyFieldCells) {
  FieldCoverageMap map(makeConfig());
  size_t field_cells = 0;
  for (size_t index = 0; index < map.size(); index++) {
    field_cells += map.isOutOfField(index) ? 0 : 1;
  }
  EXPECT_NEAR(map.totalInterest(), static_cast<double>(field_cells), 1e-9);
}

TEST(FieldCoverageMap, TotalInterestDropsAsTheFieldIsObserved) {
  FieldCoverageMap map(makeConfig());
  const double before = map.totalInterest();
  for (size_t index = 0; index < map.size(); index++) {
    map.observe(index, 1.0);
  }
  EXPECT_LT(map.totalInterest(), before);
  EXPECT_NEAR(map.totalInterest(), 0.0, 1e-9);
}

TEST(FieldCoverageMap, ResetForgetsEveryObservation) {
  FieldCoverageMap map(makeConfig());
  const double before = map.totalInterest();
  map.observe(0, 1.0);
  map.reset();
  EXPECT_DOUBLE_EQ(map.totalInterest(), before);
}
