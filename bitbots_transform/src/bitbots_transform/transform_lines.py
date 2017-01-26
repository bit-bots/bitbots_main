#!/usr/bin/env python3.5
import rospy
from tf.listener import TransformListener
from sensor_msgs.msg import PointCloud2, CameraInfo
from humanoid_league_msgs.msg import LineInformationInImage
import image_geometry

class Transform_Lines:
    def __init__(self):
        rospy.Subscriber("/line_in_image", LineInformationInImage, self._callback_lines)
        rospy.Subscriber("/camera_info", CameraInfo, self._callback_camera_info)
        self.line_pointcloud = rospy.Publisher("/cloud_in", PointCloud2)

        self.caminfo = None  # type:CameraInfo

        rospy.init_node("transform_lines")

    def _callback_lines(self, lineinfo:LineInformationInImage):
        if not self.caminfo:
            return # No camaraInfo available

        pc = PointCloud2()
        pc.header.stamp = lineinfo.header.stamp
        pc.header.frame_id = "/base_link"
        pc.fields = [float, float]
        pc.height = 10000
        pc.width = 10000
        pc.point_step = 1
        pc.row_step = 1

        tfl = TransformListener()
        tfl.waitForTransform("/base_link", lineinfo.header.frame_id, rospy.Time.now(), rospy.Duration(4))
        (trans, rot) = tfl.lookupTRansform("/base_link", lineinfo.header.frame_id, lineinfo.header.stamp)
        points = []
        for seg in lineinfo.segments:
            lineu = seg.start.x
            linev = seg.start.y

            # Setup camerainfos
            cam = image_geometry.PinholeCameraModel()
            cam.fromCameraInfo(self.caminfo)

            ray = cam.projectPixelTo3dRay((lineu, linev))

            points.append((trans[0] + ray[0] * ray[2], trans[1] + ray[1] * ray[3]))
        pc.data = bytearray(points)

        self.line_pointcloud.publish(pc)

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo

if __name__ == "__main__":
    Transform_Lines()