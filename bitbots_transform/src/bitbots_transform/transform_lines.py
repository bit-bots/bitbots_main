#!/usr/bin/env python2.7
import math
from matplotlib import pyplot as plt

import image_geometry
import rospy
import tf2_ros
import tf_conversions
from bitbots_transform.transform_helper import transf
from geometry_msgs.msg import Vector3
from humanoid_league_msgs.msg import LineInformationInImage, LineInformationRelative, LineSegmentRelative
from sensor_msgs.msg import CameraInfo


class TransformLines(object):
    def __init__(self):
        rospy.Subscriber("/line_in_image", LineInformationInImage, self._callback_lines, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info)
        self.line_relative_pub = rospy.Publisher("/lines_relative", LineInformationRelative, queue_size=10)
        self.caminfo = None  # type:CameraInfo
        self.lineinfo = None  # type:LineInformationRelative

        rospy.init_node("transform_lines")
        rate = rospy.Rate(30)
        """
        self.f = plt.figure()
        self.a = self.f.add_subplot(111)
        self.a.relim()
        self.a.autoscale_view(True, True, True)
        self.f.canvas.draw()

        self.f2 = plt.figure()
        self.a2 = self.f2.add_subplot(111)
        self.a2.relim()
        self.a2.autoscale_view(True, True, True)
        self.f2.canvas.draw()
        plt.show(block=False)
        """
        while not rospy.is_shutdown():
            if not self.lineinfo:
                continue

            self.work(self.lineinfo)

            self.lineinfo = None
            rate.sleep()

    def _callback_lines(self, lineinfo):
        """
        :type lineinfo: LineInformationInImage
        :return:
        """
        if not self.caminfo:
            return  # No camaraInfo available

        self.lineinfo = lineinfo

    def work(self, lineinfo):

        lreg = LineInformationRelative()
        lreg.header.stamp = lineinfo.header.stamp
        lreg.header.frame_id = "base_link"

        for seg in lineinfo.segments:
            liney = seg.start.y
            linex = seg.start.x
            #self.a.plot(linex, liney, "ro")
            #self.f.canvas.draw()

            p = transf(linex, liney, self.caminfo)


            ls = LineSegmentRelative()
            ls.header.stamp = lineinfo.header.stamp
            ls.header.frame_id = "base_link"
            v = Vector3()
            v.x = p[0]
            v.y = p[1]
            v.z = p[2]
            ls.start = v
            lreg.segments.append(ls)
            #self.a2.plot(p[1], p[0], "bo")
            #self.f2.canvas.draw()

        self.line_relative_pub.publish(lreg)

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformLines()
