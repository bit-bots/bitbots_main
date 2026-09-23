#include <algorithm>
#include <bitbots_head_mover/active_vision_debug.hpp>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace bitbots_head_mover {

namespace {

/// Map a score in [0, 1] to a colour running from blue over green to red.
///
/// Blue is a poor candidate, red is a good one. Using a full hue sweep rather
/// than a brightness ramp keeps the individual candidates distinguishable even
/// where many of them overlap.
cv::Scalar scoreColor(double normalized) {
  const double clamped = std::clamp(normalized, 0.0, 1.0);
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((1.0 - clamped) * 120.0, 255, 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b pixel = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(pixel[0], pixel[1], pixel[2]);
}

/// Normalize the candidate scores into [0, 1] for colouring.
///
/// The absolute scores depend entirely on the configured weights, so they are
/// rescaled against the spread actually observed in this planning cycle.
std::vector<double> normalizedScores(const ActiveVisionResult& result) {
  std::vector<double> normalized(result.scores.size(), 0.5);
  if (result.scores.empty()) {
    return normalized;
  }

  double lowest = result.scores.front().total;
  double highest = result.scores.front().total;
  for (const auto& score : result.scores) {
    lowest = std::min(lowest, score.total);
    highest = std::max(highest, score.total);
  }

  const double span = highest - lowest;
  if (span <= 0.0) {
    return normalized;
  }
  for (size_t i = 0; i < result.scores.size(); i++) {
    normalized[i] = (result.scores[i].total - lowest) / span;
  }
  return normalized;
}

/// Where the optical axis of a camera pose meets the ground plane.
///
/// Returns false when the camera looks at or above the horizon, in which case
/// there is no ground intersection to draw.
bool groundIntersection(const Eigen::Isometry3d& camera_pose, Eigen::Vector3d& intersection) {
  // In an optical frame the viewing direction is the z axis
  const Eigen::Vector3d direction = camera_pose.linear().col(2);
  const Eigen::Vector3d origin = camera_pose.translation();
  if (direction.z() >= -1e-6) {
    return false;
  }
  const double distance = -origin.z() / direction.z();
  intersection = origin + distance * direction;
  return true;
}

}  // namespace

nav_msgs::msg::OccupancyGrid coverageGrid(const FieldCoverageMap& coverage, const std::string& frame_id,
                                          const builtin_interfaces::msg::Time& stamp) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.header.stamp = stamp;

  const FieldCoverageConfig& config = coverage.config();
  grid.info.resolution = config.cell_size;
  grid.info.width = static_cast<uint32_t>(coverage.cellsX());
  grid.info.height = static_cast<uint32_t>(coverage.cellsY());

  // The grid origin is the lower left corner of the lower left cell, while the
  // stored centers sit in the middle of their cells
  grid.info.origin.position.x = -(static_cast<double>(coverage.cellsX()) * config.cell_size) / 2.0;
  grid.info.origin.position.y = -(static_cast<double>(coverage.cellsY()) * config.cell_size) / 2.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.reserve(coverage.size());
  for (size_t index = 0; index < coverage.size(); index++) {
    grid.data.push_back(static_cast<int8_t>(std::lround(coverage.interest(index) * 100.0)));
  }
  return grid;
}

visualization_msgs::msg::MarkerArray candidateMarkers(const ActiveVisionResult& result,
                                                      const ActiveVision& active_vision,
                                                      const Eigen::Isometry3d& robot_pose, const std::string& frame_id,
                                                      const builtin_interfaces::msg::Time& stamp) {
  visualization_msgs::msg::MarkerArray markers;

  // Clear whatever the previous cycle drew, otherwise candidates from a cycle
  // with more samples would linger forever
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  if (!result.valid || result.candidates.empty()) {
    return markers;
  }

  const std::vector<double> normalized = normalizedScores(result);
  const ActiveVisionScorer scorer(active_vision.kinematics(), active_vision.camera(), active_vision.world(),
                                  active_vision.coverage(), robot_pose);

  // All candidates share one sphere list, so a single marker carries the whole
  // sampled distribution instead of one marker per candidate
  visualization_msgs::msg::Marker points;
  points.header.frame_id = frame_id;
  points.header.stamp = stamp;
  points.ns = "active_vision_candidates";
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = points.scale.y = points.scale.z = 0.08;

  for (size_t index = 0; index < result.candidates.size(); index++) {
    const auto camera_pose = scorer.cameraPoseInMap(result.candidates[index].target, robot_pose);
    Eigen::Vector3d point;
    // A pose the kinematics could not resolve simply has no ground point to
    // draw; the planner reports that failure itself
    if (!camera_pose || !groundIntersection(*camera_pose, point)) {
      continue;
    }
    geometry_msgs::msg::Point message_point;
    message_point.x = point.x();
    message_point.y = point.y();
    message_point.z = point.z();
    points.points.push_back(message_point);

    const cv::Scalar color = scoreColor(normalized[index]);
    std_msgs::msg::ColorRGBA rgba;
    rgba.b = static_cast<float>(color[0] / 255.0);
    rgba.g = static_cast<float>(color[1] / 255.0);
    rgba.r = static_cast<float>(color[2] / 255.0);
    rgba.a = 0.6f;
    points.colors.push_back(rgba);
  }

  if (!points.points.empty()) {
    markers.markers.push_back(points);
  }

  // Draw the selected target on top as a larger, distinct sphere
  if (result.selected < result.candidates.size()) {
    const auto camera_pose = scorer.cameraPoseInMap(result.candidates[result.selected].target, robot_pose);
    Eigen::Vector3d point;
    if (camera_pose && groundIntersection(*camera_pose, point)) {
      visualization_msgs::msg::Marker selected;
      selected.header.frame_id = frame_id;
      selected.header.stamp = stamp;
      selected.ns = "active_vision_selected";
      selected.id = 0;
      selected.type = visualization_msgs::msg::Marker::SPHERE;
      selected.action = visualization_msgs::msg::Marker::ADD;
      selected.pose.orientation.w = 1.0;
      selected.pose.position.x = point.x();
      selected.pose.position.y = point.y();
      selected.pose.position.z = point.z();
      selected.scale.x = selected.scale.y = selected.scale.z = 0.2;
      selected.color.r = selected.color.g = selected.color.b = 1.0f;
      selected.color.a = 1.0f;
      markers.markers.push_back(selected);
    }
  }

  return markers;
}

cv::Mat jointSpaceDebugImage(const ActiveVisionResult& result, const HeadLimits& limits, int size) {
  cv::Mat image(size, size, CV_8UC3, cv::Scalar(30, 30, 30));

  const double yaw_span = limits.yaw.upper - limits.yaw.lower;
  const double pitch_span = limits.pitch.upper - limits.pitch.lower;
  if (!(yaw_span > 0.0) || !(pitch_span > 0.0)) {
    return image;
  }

  // Yaw runs left to right, pitch runs top to bottom, so looking up is at the
  // top of the image the way one would expect
  const auto toPixel = [&](const HeadPosition& position) {
    const double x = (position.yaw - limits.yaw.lower) / yaw_span * (size - 1);
    const double y = (position.pitch - limits.pitch.lower) / pitch_span * (size - 1);
    return cv::Point(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)));
  };

  // Axes through the zero position, so the neutral head pose is easy to find
  cv::line(image, toPixel({0.0, limits.pitch.lower}), toPixel({0.0, limits.pitch.upper}), cv::Scalar(70, 70, 70), 1);
  cv::line(image, toPixel({limits.yaw.lower, 0.0}), toPixel({limits.yaw.upper, 0.0}), cv::Scalar(70, 70, 70), 1);

  if (result.candidates.empty()) {
    return image;
  }

  const std::vector<double> normalized = normalizedScores(result);

  // Draw the selected candidate last so it ends up on top of the others
  std::vector<size_t> order;
  order.reserve(result.candidates.size());
  for (size_t index = 0; index < result.candidates.size(); index++) {
    if (index != result.selected) {
      order.push_back(index);
    }
  }
  if (result.selected < result.candidates.size()) {
    order.push_back(result.selected);
  }

  for (size_t index : order) {
    const bool selected = index == result.selected;
    const cv::Scalar color = selected ? cv::Scalar(255, 255, 255) : scoreColor(normalized[index]);
    cv::circle(image, toPixel(result.candidates[index].target), selected ? 5 : 2, color, cv::FILLED, cv::LINE_AA);
  }

  return image;
}

}  // namespace bitbots_head_mover
