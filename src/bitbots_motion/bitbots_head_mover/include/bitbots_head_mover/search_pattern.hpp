#pragma once

#include <bitbots_head_mover/types.hpp>
#include <vector>

/// Generation of the open-loop head search patterns.
///
/// The patterns are boustrophedon scans: the head sweeps horizontally along a
/// scan line, steps to the next line, and sweeps back in the opposite
/// direction. All angles handled here are in the unit in which the patterns are
/// configured, which is degrees; the conversion to radians happens when the
/// pattern is turned into a trajectory.
namespace bitbots_head_mover {

/// Convert a scan line index to its angle.
///
/// The lines are distributed evenly between the two given angles, with line 0
/// at min_angle and the last line at max_angle. A pattern with a single scan
/// line has no span to distribute and collapses onto min_angle.
double lineAngle(int line, int line_count, double min_angle, double max_angle);

/// Linearly interpolate between two yaw values at a constant pitch.
///
/// Returns the interpolated positions ordered from min_yaw towards max_yaw,
/// excluding min_yaw itself and including max_yaw. Returns an empty vector if no
/// interpolation was requested.
std::vector<HeadPosition> interpolatedSteps(int steps, double pitch, double min_yaw, double max_yaw);

/// Generate the keyframes of a parameterized search pattern.
///
/// The scan starts at the bottom line and alternates its horizontal direction
/// on every line. Keyframes on the lowest line are scaled towards the center by
/// reduce_last_scanline, because looking far to the side while looking down
/// makes the head collide with the body.
std::vector<HeadPosition> generatePattern(int line_count, double max_horizontal_angle_left,
                                          double max_horizontal_angle_right, double max_vertical_angle_up,
                                          double max_vertical_angle_down, double reduce_last_scanline = 1.0,
                                          int interpolation_steps = 0);

}  // namespace bitbots_head_mover
