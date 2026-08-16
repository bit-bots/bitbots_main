#pragma once

#include <bitbots_head_mover/active_vision.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

/// Visualization of what the active vision planner is doing.
namespace bitbots_head_mover {

/// Render the coverage map as an occupancy grid.
///
/// The cell values carry the interest of a cell, so an unobserved part of the
/// field shows up as occupied and a freshly seen one as free. Cells outside the
/// field carry no interest and therefore always read as free.
nav_msgs::msg::OccupancyGrid coverageGrid(const FieldCoverageMap& coverage, const std::string& frame_id,
                                          const builtin_interfaces::msg::Time& stamp);

/// Render the sampled candidates and the selected one as markers.
///
/// Each candidate is drawn as the ground track its optical axis sweeps over,
/// coloured by its score, which makes it visible at a glance whether the planner
/// is considering the right part of the field. The selected candidate is drawn
/// on top in a distinct colour.
visualization_msgs::msg::MarkerArray candidateMarkers(const ActiveVisionResult& result,
                                                      const ActiveVision& active_vision,
                                                      const Eigen::Isometry3d& robot_pose, const std::string& frame_id,
                                                      const builtin_interfaces::msg::Time& stamp);

/// Plot the sampled trajectories in the yaw and pitch joint space.
///
/// The horizontal axis is yaw and the vertical axis is pitch, each spanning the
/// configured joint limits. Every candidate is drawn as a polyline coloured by
/// its score, with the selected one highlighted, so the shape of the search and
/// the score landscape can be judged directly rather than inferred from numbers.
cv::Mat jointSpaceDebugImage(const ActiveVisionResult& result, const HeadLimits& limits, double horizon, int size);

}  // namespace bitbots_head_mover
