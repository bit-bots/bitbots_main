#ifndef BITBOTS_SPLINES_ABSTRACTVISUALIZER_H
#define BITBOTS_SPLINES_ABSTRACTVISUALIZER_H

#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <bitbots_splines/AbstractEngine.h>

class AbstractVisualizer {
public:
    visualization_msgs::Marker get_marker(const tf2::Vector3& position, const std::string& frame) {
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
};

#endif //BITBOTS_SPLINES_ABSTRACTVISUALIZER_H
