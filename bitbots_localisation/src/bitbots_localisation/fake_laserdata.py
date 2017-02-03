#!/usr/bin/env python2
import math
from cv2 import norm

import numpy as np
from numpy import dot
import collections
import rospy
from geometry_msgs.msg import PointStamped
from humanoid_league_msgs.msg import LineInformationInImage, LineInformationRelative
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
import tf
from tf.listener import TransformListener


def angle_b(a):
    b = (0, 1)
    arccos_input = dot(a, b) / norm(a) / norm(b)
    arccos_input = 1.0 if arccos_input > 1.0 else arccos_input
    arccos_input = -1.0 if arccos_input < -1.0 else arccos_input
    return math.acos(arccos_input)


def polar(x,y):
  return math.hypot(x,y),math.degrees(math.atan2(y,x))%360


class FakeLaser(object):
    def __init__(self):
        self.sub_line = rospy.Subscriber("/lines_relative", LineInformationRelative, self._callback_lines, queue_size=1)
        self.pub_laser = rospy.Publisher("/scan", LaserScan, queue_size=200)
        rospy.init_node("bitbots_Fakelaser")
        self.tfl = TransformListener()


        self.f = plt.figure()
        self.a = self.f.add_subplot(111, projection='polar')
        self.a.relim()
        self.a.autoscale_view(True, True, True)
        self.f.canvas.draw()
        plt.show(block=False)
        self.newest = collections.deque()


        while not rospy.is_shutdown():
            if len(self.newest) > 0:
                newest = self.newest.popleft()
                self.a.plot(newest[0], newest[1], "go")
                self.f.canvas.draw()
            else:
                rospy.sleep(0.1)

        rospy.spin()

    def _callback_lines(self, lines):
        linepoints = [(x.start.x, x.start.y) for x in lines.segments]
        lp = [polar(p[0], p[1]) for p in linepoints]
        print(lp)
        for deg in range(360):

            ls = LaserScan()
            ls.header.frame_id = lines.header.frame_id
            ls.header.stamp = lines.header.stamp
            ls.range_max = 12.0

            ls.angle_increment = np.deg2rad(1)
            ls.angle_min = np.deg2rad(deg - 1)
            ls.angle_max = np.deg2rad(deg)
            #for scan in filter(angle_b, linepoints):
            #print([round(math.degrees(angle_b(a))) for a in linepoints])
            se = [a for a in lp if abs(a[1] - deg) < 0.5]
            #print(se)
            if len(se) > 0:
                for scan in se:
                    ls.ranges.append(scan[0])
                    self.newest.append((ls.angle_min, scan[0]))
                print(ls.ranges)
                self.pub_laser.publish(ls)


if __name__ == "__main__":
    FakeLaser()
