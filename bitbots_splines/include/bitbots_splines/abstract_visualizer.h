#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTVISUALIZER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTVISUALIZER_H_

#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <bitbots_splines/abstract_engine.h>

namespace bitbots_splines {

class AbstractVisualizer {
 protected:
  /**
   * Utility function to create a visualization marker for a position with default properties.
   * @param position The position of the marker
   * @param frame The frame in which the position is given
   * @return The visualization marker
   */
  visualization_msgs::Marker getMarker(const tf2::Vector3 &position, const std::string &frame) {
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
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
   * @param splines A container containing the splines to be displayed
   * @param frame The frame in which the splines are given
   * @param key_x The key to access the x spline from the splines parameter
   * @param key_y The key to access the y spline from the splines parameter
   * @param key_z The key to access the z spline from the splines parameter
   * @param smoothness The smoothness of the splines
   * @return The visualization marker
   */
  visualization_msgs::Marker getPath(const Trajectories &splines,
                                     const std::string &frame,
                                     const std::string &key_x,
                                     const std::string &key_y,
                                     const std::string &key_z,
                                     const double smoothness) {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.01;
    marker.color.a = 1;

    std::vector<double> times = splines.getTimes();
    double first_time = times[0];
    double last_time = times[times.size() - 1];

    for (double i = first_time; i <= last_time; i += (last_time - first_time) / smoothness) {
      geometry_msgs::Point point;
      point.x = splines.get(key_x).pos(i);
      point.y = splines.get(key_y).pos(i);
      point.z = splines.get(key_z).pos(i);

      marker.points.push_back(point);
    }

    return marker;
  }
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTVISUALIZER_H_
