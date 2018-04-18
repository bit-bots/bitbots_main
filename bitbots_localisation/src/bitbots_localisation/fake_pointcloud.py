#!/usr/bin/env python
import sys
import rospy
from humanoid_league_msgs.msg import LineInformationRelative
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2




class FakePC2:
    def __init__(self):
        rospy.init_node("bitbots_FakePC2")
        self.sub_line = rospy.Subscriber("line_relative", LineInformationRelative, self._callback_lines, queue_size=1)
        self.pub_laser = rospy.Publisher("cloud_in", PointCloud2, queue_size=200)

        rospy.spin()

    def _callback_lines(self, lines):
        points = []
        for line in lines.segments:
            points.append([line.start.x, line.start.y, line.start.z])
           # points.append(line.start.y)
           # points.append(line.start.z)

        cloud = pc2.create_cloud_xyz32(lines.header, points)
        #rospy.logwarn(cloud)
        self.pub_laser.publish(cloud)


if __name__ == "__main__":
    FakePC2()
