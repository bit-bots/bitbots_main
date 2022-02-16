#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import LineInformationRelative
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


class LineViz:
    def __init__(self):
        rclpy.init(args=None)

        rospy.Subscriber("line_relative", LineInformationRelative, self.callback, queue_size=1)
        self.marker_pub = self.create_publisher(Marker, "robocup_markers", 1)

        self.id = 0

        rclpy.spin(self)

    def callback(self, msg):
        mark = Marker()
        mark.header = msg.header
        mark.ns = "lines"
        mark.id = self.id
        self.id += 1
        mark.type = Marker.SPHERE_LIST
        mark.action = Marker.ADD
        mark.scale = Vector3(0.03, 0.03, 0.03)
        mark.lifetime = Duration(seconds=3.5)
        mark.color = ColorRGBA(1, 0, 0, 1)

        for segment in msg.segments:
            mark.points.append(segment.start)
            mark.points.append(segment.end)

        self.marker_pub.publish(mark)


if __name__ == "__main__":
    LineViz()
