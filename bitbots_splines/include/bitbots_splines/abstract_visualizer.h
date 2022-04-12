#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_

#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <bitbots_splines/pose_spline.h>

namespace bitbots_splines {

class AbstractVisualizer {
 protected:
  int id = 0;
  /**
   * Utility function to create a visualization marker for a position with default properties.
   * @param position The position of the marker
   * @param frame The frame in which the position is given
   * @return The visualization marker
   */
  visualization_msgs::msg::Marker getMarker(const tf2::Vector3 &position, const std::string &frame, rclcpp::Node* node) {
    visualization_msgs::msg::Marker marker;

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.lifetime = rclcpp::Duration::from_nanoseconds(1e9*1000);
    marker.frame_locked = false;
    marker.header.frame_id = frame;
    marker.header.stamp = node->now();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1;

    return marker;
  }

  /**
   * Utility function to create a visualization marker for a trajectory with default properties.
   * @param spline A PoseSpline containing the splines to be displayed
   * @param frame The frame in which the splines are given
   * @param smoothness The smoothness of the splines
   * @return The visualization markers
   */
  visualization_msgs::msg::MarkerArray getPath(bitbots_splines::PoseSpline &spline,
                                     const std::string &frame,
                                     const double smoothness,
                                     rclcpp::Node::SharedPtr node) {

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker base_marker;
    base_marker.action = visualization_msgs::msg::Marker::ADD;
    base_marker.lifetime = rclcpp::Duration::from_nanoseconds(1e9*1000);
    base_marker.frame_locked = true;
    base_marker.header.frame_id = frame;
    base_marker.header.stamp = node->now();
    base_marker.pose.orientation.w = 1;
    base_marker.color.a = 1;

    // Marker for spline path
    visualization_msgs::msg::Marker path_marker = base_marker;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.id = id++;
    path_marker.scale.x = 0.01;

    // Take length of spline, assuming values at the beginning and end are set for x
    double first_time = spline.x()->min();
    double last_time = spline.x()->max();

    // Iterate over splines, get points everywhere
    // Taking the manually set points is not enough because velocities and accelerations influence the curve
    for (double i = first_time; i <= last_time; i += (last_time - first_time) / smoothness) {
      geometry_msgs::msg::Point point;
      point.x = spline.x()->pos(i);
      point.y = spline.y()->pos(i);
      point.z = spline.z()->pos(i);

      path_marker.points.push_back(point);
    }
    marker_array.markers.push_back(path_marker);

    // Marker for spline orientation
    // Get all times of manually added points
    std::list<double> times;
    for (const SmoothSpline::Point &p : spline.roll()->points()) {
      times.push_back(p.time);
    }
    for (const SmoothSpline::Point &p : spline.pitch()->points()) {
      times.push_back(p.time);
    }
    for (const SmoothSpline::Point &p : spline.yaw()->points()) {
      times.push_back(p.time);
    }
    times.sort();
    times.unique();

    visualization_msgs::msg::Marker orientation_marker = base_marker;
    orientation_marker.scale.x = 0.05;
    orientation_marker.scale.y = 0.005;
    orientation_marker.scale.z = 0.005;
    orientation_marker.type = visualization_msgs::msg::Marker::ARROW;
    for (double time : times) {
      orientation_marker.id = id++;
      orientation_marker.pose.position = spline.getGeometryMsgPosition(time);

      // We use three axes for every orientation
      // x
      orientation_marker.color.b = 0;
      orientation_marker.color.r = 1;
      tf2::Quaternion x_rotation = spline.getOrientation(time) * tf2::Vector3(1, 0, 0);
      orientation_marker.pose.orientation = tf2::toMsg(x_rotation.normalize());
      marker_array.markers.push_back(orientation_marker);

      // y
      orientation_marker.color.r = 0;
      orientation_marker.color.g = 1;
      tf2::Quaternion y_rotation = spline.getOrientation(time) * tf2::Vector3(1, 1, 0);
      orientation_marker.pose.orientation = tf2::toMsg(y_rotation.normalize());
      orientation_marker.id = id++;
      marker_array.markers.push_back(orientation_marker);

      // z
      orientation_marker.color.g = 0;
      orientation_marker.color.b = 1;
      tf2::Quaternion z_rotation = spline.getOrientation(time) * tf2::Vector3(1, 0, 1);
      orientation_marker.pose.orientation = tf2::toMsg(z_rotation.normalize());
      orientation_marker.id = id++;
      marker_array.markers.push_back(orientation_marker);
    }
    return marker_array;
  }
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_
