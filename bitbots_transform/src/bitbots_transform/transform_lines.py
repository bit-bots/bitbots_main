#!/usr/bin/env python2.7
import rospy
import tf2_ros
from geometry_msgs.msg import Point32, Vector3
from sensor_msgs.msg import PointCloud2, CameraInfo, PointCloud, PointField
from sensor_msgs.point_cloud2 import create_cloud
from humanoid_league_msgs.msg import LineInformationInImage, LineInformationRelative, LineSegmentRelative
import image_geometry
from std_msgs.msg import Header


class TransformLines(object):
    def __init__(self):
        rospy.Subscriber("/line_in_image", LineInformationInImage, self._callback_lines, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info)
        #self.line_pointcloud = rospy.Publisher("/cloud_in", PointCloud2, queue_size=10)
        self.line_relative_pub = rospy.Publisher("/lines_relative", LineInformationRelative, queue_size=10)
        self.caminfo = None  # type:CameraInfo
        self.lineinfo = None  # type:LineInformationRelative

        rospy.init_node("transform_lines")
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if not self.lineinfo:
                continue
            try:
                self.work(self.lineinfo)
            except:
                pass
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
        # pit = lineinfo.header.stamp
        pit = rospy.Time(0)
        trans = tfbuffer.lookup_transform("base_link", "L_CAMERA", pit, rospy.Duration(0.1))
        #points = []
        lreg = LineInformationRelative()
        lreg.header.stamp = lineinfo.header.stamp
        lreg.header.frame_id = lineinfo.header.frame_id

        for seg in lineinfo.segments:
            lineu = seg.start.x
            linev = seg.start.y

            # Setup camerainfos
            cam = image_geometry.PinholeCameraModel()
            cam.fromCameraInfo(self.caminfo)

            ray = cam.projectPixelTo3dRay((lineu, linev))

            p = [trans.transform.translation.x + ray[0] * ray[2]*1000, trans.transform.translation.y + ray[1] * ray[2]*1000, 0]
            # points.append(p)

            # use poincloud
            # header = Header()
            # header.stamp = lineinfo.header.stamp
            # header.frame_id = "base_link"
            # pfs = None  # TODO
            # pc2 = create_cloud(header, pfs, points)
            #
            # self.line_pointcloud.publish(pc2)

            ls = LineSegmentRelative()
            ls.header.stamp = lineinfo.header.stamp
            ls.header.frame_id = "base_link"
            v = Vector3()
            v.x = p[0]
            v.y = p[1]
            v.z = p[2]
            ls.start = v
            lreg.segments.append(ls)

        self.line_relative_pub.publish(lreg)

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformLines()
