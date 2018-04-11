#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallRelative, BallsInImage, \
    LineInformationInImage, LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
import tf2_ros
import math
from tf2_geometry_msgs import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
import numpy as np


class TransformBall(object):
    def __init__(self):
        rospy.init_node("bitbots_transformer")

        rospy.Subscriber("ball_in_image", BallsInImage, self._callback_ball, queue_size=1)
        #rospy.Subscriber("line_in_image", LineInformationInImage, self._callback_lines, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info, queue_size=1)

        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=10)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=10)
        self.line_relative_pub = rospy.Publisher("line_relative", LineInformationRelative, queue_size=10)

        self.caminfo = None


        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_height = 0.1
        rospy.spin()

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo

    def _callback_ball(self, msg):
        br = BallRelative()
        br.header.stamp = msg.header.stamp
        br.header.frame_id = "L_CAMERA"

        for ball in msg.candidates:
            br.ball_relative = self.transform(ball.center, msg.header.stamp, self.ball_height)
            br.confidence = ball.confidence
            print br.ball_relative.x
            print br.ball_relative.y
            print br.ball_relative.z

            if br.ball_relative is not None:
                self.ball_relative_pub.publish(br)

    def _callback_lines(self, msg):
        line = LineInformationRelative()
        line.header.stamp = msg.header.stamp
        line.header.frame_id = "L_CAMERA"

        for seg in msg.segments:
            rel_seg = LineSegmentRelative()
            rel_seg.start = self.transform(seg.start, msg.header.stamp, 0)
            rel_seg.end = self.transform(seg.end, msg.header.stamp, 0)

            rel_seg.confidence = seg.confidence

            # only proceed if all transformations were successful
            if rel_seg.start is not None and rel_seg.end is not None:
                line.segments.append(rel_seg)

        for circle in msg.circles:
            rel_circle = LineCircleRelative()
            rel_circle.left = self.transform(circle.left, msg.header.stamp, 0)
            rel_circle.middle = self.transform(circle.middle, msg.header.stamp, 0)
            rel_circle.right = self.transform(circle.right, msg.header.stamp, 0)

            rel_circle.confidence = circle.confidence

            # only proceed if all transformations were successful
            if rel_circle.left is not None and rel_circle.middle is not None and rel_circle.right is not None:
                line.circles.append(rel_circle)

        for intersection in msg.intersections:
            rel_inter = LineIntersectionRelative()
            broken = False
            for segment in intersection.segments:
                rel_seg = LineSegmentRelative()
                rel_seg.start = self.transform(segment.start, msg.header.stamp, 0)
                rel_seg.end = self.transform(segment.end, msg.header.stamp, 0)

                rel_seg.confidence = seg.confidence

                if rel_seg.start is not None and rel_seg.end is not None:
                    rel_inter.segments.append(rel_seg)
                else:
                    broken = True
                    break

            rel_inter.type = intersection.type
            rel_inter.confidence = intersection.confidence

            if not broken:
                line.intersections.append(rel_inter)

        self.line_relative_pub.publish(line)




    def transform(self, object, stamp, object_height):

        # normalized camera coordinates coordinate system
        #  -1,0 ------------- 1,1
        #       |           |
        #       |    0,0    |
        #       |           |
        # -1,-1 ------------- 1,0
        normalized_x = 2 * (float(object.x)-(float(self.caminfo.width)/2))/float(self.caminfo.width)
        normalized_y = (-2 * (float(object.y)-(float(self.caminfo.height)/2))/float(self.caminfo.height)) * \
                       (float(self.caminfo.height)/float(self.caminfo.width))

        object_height = rospy.get_param("/object_height", object_height)
        #TODO get fov from camerainfo
        fov = 1.012300
        focal_length = 1.0 / math.tan(fov/2)


        point_on_image = np.array([focal_length, -normalized_x, normalized_y])

        tf_right = self.tf_buffer.lookup_transform("L_CAMERA", "right_foot_sole_link", stamp)
        tf_left = self.tf_buffer.lookup_transform("L_CAMERA", "left_foot_sole_link", stamp)

        len_r = math.sqrt(tf_right.transform.translation.x ** 2 +
                          tf_right.transform.translation.y ** 2 +
                          tf_right.transform.translation.z ** 2)
        len_l = math.sqrt(tf_left.transform.translation.x ** 2 +
                          tf_left.transform.translation.y ** 2 +
                          tf_left.transform.translation.z ** 2)

        if len_r > len_l:
            ground_foot = "right_foot_sole_link"
        else:
            ground_foot = "left_foot_sole_link"


        field_normal = PointStamped()
        field_normal.header.frame_id = ground_foot
        field_normal.header.stamp = stamp
        field_normal.point.x = 0.0
        field_normal.point.y = 0.0
        field_normal.point.z = 1.0
        field_normal = self.tf_buffer.transform(field_normal, "L_CAMERA")

        field_point = PointStamped()
        field_point.header.frame_id = ground_foot
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        field_point = self.tf_buffer.transform(field_point, "L_CAMERA")

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])
        field_normal = field_point - field_normal

        return line_plane_collision(field_normal, field_point, point_on_image)


def line_plane_collision(planeNormal, planePoint, rayDirection,  epsilon=1e-3):
    ndotu = planeNormal.dot(rayDirection)
    # checking if intersection can be found at reasonable distance or at all
    if abs(ndotu) < epsilon:
        return None

    si = -planeNormal.dot(- planePoint) / ndotu
    intersection = -planePoint + si * rayDirection + planePoint
    return Point(intersection[0], intersection[1], intersection[2])




if __name__ == "__main__":
    TransformBall()
