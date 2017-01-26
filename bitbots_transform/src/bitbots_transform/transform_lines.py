#!/usr/bin/env python2.7
import rospy
import tf2_ros
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2, CameraInfo, PointCloud, PointField
from sensor_msgs.point_cloud2 import create_cloud
from humanoid_league_msgs.msg import LineInformationInImage
import image_geometry
from std_msgs.msg import Header


class TransformLines(object):
    def __init__(self):
        rospy.Subscriber("/line_in_image", LineInformationInImage, self._callback_lines)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info)
        self.line_pointcloud = rospy.Publisher("/cloud_in", PointCloud2, queue_size=10)

        self.caminfo = None  # type:CameraInfo

        rospy.init_node("transform_lines")
        rospy.spin()

    def _callback_lines(self, lineinfo):
        """
        :type lineinfo: LineInformationInImage
        :return:
        """
        if not self.caminfo:
            return  # No camaraInfo available

        tfbuffer = tf2_ros.Buffer()
        tfl = tf2_ros.TransformListener(tfbuffer)
        trans = tfbuffer.lookup_transform("base_link", "L_CAMERA", rospy.Time.now(), rospy.Duration(1))
        points = []
        for seg in lineinfo.segments:
            lineu = seg.start.x
            linev = seg.start.y

            # Setup camerainfos
            cam = image_geometry.PinholeCameraModel()
            cam.fromCameraInfo(self.caminfo)

            ray = cam.projectPixelTo3dRay((lineu, linev))

            point = [trans.transform.translation.x + ray[0] * ray[2], trans.transform.translation.y + ray[1] * ray[2], 0]
            points.append(point)

        header = Header()
        header.stamp = lineinfo.header.stamp
        header.frame_id = "base_link"
        pfs = [PointField.FLOAT32, PointField.FLOAT32, PointField.FLOAT32]
        pc2 = create_cloud(header, pfs, points)

        self.line_pointcloud.publish(pc2)

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformLines()
