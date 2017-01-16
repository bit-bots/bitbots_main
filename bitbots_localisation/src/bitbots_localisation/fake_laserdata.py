#!/usr/bin/env python3
import math
from cv2 import norm

import numpy as np
from numpy import dot

import rospy
from humanoid_league_msgs.msg import LineInformationInImage
from sensor_msgs.msg import LaserScan


class FakeLaser:
    def __init__(self):
        self.sub_line = rospy.Subscriber("/lines_relative", self._callback_lines)
        self.pub_laser = rospy.Publisher("/laser_scan_data", LaserScan)

    def _callback_lines(self, lines: LineInformationInImage):
        linepoints = [(x.start.x, x.start.y) for x in lines.segments]
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


def angle_b(a):
    b = (1, 0)
    arccos_input = dot(a, b) / norm(a) / norm(b)
    arccos_input = 1.0 if arccos_input > 1.0 else arccos_input
    arccos_input = -1.0 if arccos_input < -1.0 else arccos_input
    return math.acos(arccos_input)


if __name__ == "__main__":
    FakeLaser()
