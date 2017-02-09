#!/usr/bin/env python2.7
import math
from matplotlib import pyplot as plt

import image_geometry
import rospy
import tf2_ros
import tf_conversions
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

        tfbuffer = tf2_ros.Buffer(cache_time=rospy.Duration(6))
        tfl = tf2_ros.TransformListener(tfbuffer)

        pit = rospy.Time(0)
        trans = tfbuffer.lookup_transform("base_link", "L_CAMERA", pit, rospy.Duration(0.1))
        lreg = LineInformationRelative()
        lreg.header.stamp = lineinfo.header.stamp
        lreg.header.frame_id = "base_link"

        for seg in lineinfo.segments:
            liney = seg.start.y
            linex = seg.start.x
            self.a.plot(linex, liney, "ro")
            self.f.canvas.draw()
            print("run2")

            # Setup camerainfos
            cam = image_geometry.PinholeCameraModel()
            cam.fromCameraInfo(self.caminfo)
            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w)
            euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
            pitch = euler[1]
            yaw = euler[2]
            print("run2.3")

            p = self.transf(yaw, pitch, linex, liney, cam)

            print(p)

            ls = LineSegmentRelative()
            ls.header.stamp = lineinfo.header.stamp
            ls.header.frame_id = "base_link"
            v = Vector3()
            v.x = p[0]
            v.y = p[1]
            v.z = p[2]
            ls.start = v
            lreg.segments.append(ls)
            self.a2.plot(p[1], p[0], "bo")
            self.f2.canvas.draw()

        self.line_relative_pub.publish(lreg)

    def transf(self, cam_tilt, cam_pan, x, y, cam):

        y = y - 600

        angle_horizontal = (x - cam.width / 2.0) / cam.width * 72.0

        angle_vertical = -(y - cam.height / 2.0) / cam.height * 72.0

        angle = cam_pan + angle_horizontal
        distance = 7 * abs(math.tan(math.radians(cam_tilt + angle_vertical)) / math.cos(math.radians(angle)))

        return distance * math.cos(math.radians(angle)), distance * math.sin(math.radians(angle)), 0

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformLines()
