#!/usr/bin/env python2
import math
from cv2 import norm

import numpy as np
from numpy import dot

import rospy
from geometry_msgs.msg import PointStamped
from humanoid_league_msgs.msg import LineInformationInImage
from sensor_msgs.msg import LaserScan

import tf
from tf.listener import TransformListener


def angle_b(a):
    b = (1, 0)
    arccos_input = dot(a, b) / norm(a) / norm(b)
    arccos_input = 1.0 if arccos_input > 1.0 else arccos_input
    arccos_input = -1.0 if arccos_input < -1.0 else arccos_input
    return math.acos(arccos_input)


class FakeLaser(object):
    def __init__(self):
        self.sub_line = rospy.Subscriber("/line_in_image", LineInformationInImage, self._callback_lines)
        self.pub_laser = rospy.Publisher("/laser_scan_data", LaserScan)
        rospy.init_node("bitbots_Fakelaser")
        self.tfl = TransformListener()

        rospy.spin()

    def _callback_lines(self, lines):
        linepoints = [(x.start.x, x.start.y) for x in lines.segments]
        relp =[]
        for x, y in linepoints:
            ps = PointStamped()
            ps.point.x = x
            ps.point.y = y
            ps.header.frame_id = lines.header.frame_id
            rospy.logwarn(lines.header.frame_id)
            self.tfl.waitForTransform("/base_link", lines.header.frame_id, rospy.Time.now(), rospy.Duration(4))
            l = self.tfl.transformPoint("/base_link", ps)
            relp.append((l.point.x, l.point.y))

        for deg in range(360):
            ls = LaserScan()
            ls.header.frame_id = lines.header.frame_id
            ls.header.stamp = lines.header.stamp

            ls.angle_increment = np.deg2rad(1)
            ls.angle_min = np.deg2rad(deg - 1)
            ls.angle_max = np.deg2rad(deg)
            for scan in filter(angle_b, linepoints):
                ls.ranges.append(math.sqrt(scan[0] ** 2 * scan[1] ** 2))
            self.pub_laser.publish(ls)


if __name__ == "__main__":
    FakeLaser()
