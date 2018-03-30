#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallRelative, BallInImage, BallsInImage, \
    LineInformationInImage, LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
import tf2_ros
import math
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import tf
import tf_conversions


class TransformBall(object):
    def __init__(self):
        rospy.Subscriber("ball_in_image", BallsInImage, self._callback_ball, queue_size=1)
        rospy.Subscriber("line_in_image", LineInformationInImage, self._callback_lines, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info, queue_size=1)

        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=10)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=10)
        self.line_relative_pub = rospy.Publisher("lines_relative", LineInformationRelative, queue_size=10)

        self.caminfo = None

        rospy.init_node("bitbots_transformer")
        rospy.set_param("/object_height", 0.1)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_height = 0.1
        rospy.sleep(1)
        rospy.spin()



    def _callback_ball(self, msg):
        #transformed = self.transform

        br = BallRelative()
        br.header.stamp = msg.header.stamp  # todo ball in image time!
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

        point_on_image = PoseStamped()
        point_on_image.header.frame_id = "L_CAMERA"
        point_on_image.header.stamp = stamp
        point_on_image.pose.position.x = focal_length
        point_on_image.pose.position.y = -normalized_x
        point_on_image.pose.position.z = normalized_y

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

        print ground_foot

        field_normal = PoseStamped()
        field_normal.header.frame_id = ground_foot
        field_normal.header.stamp = stamp
        field_normal.pose.position.x = 0.0
        field_normal.pose.position.y = 0.0
        field_normal.pose.position.z = 1.0
        field_normal = self.tf_buffer.transform(field_normal, "L_CAMERA")

        field_point = PoseStamped()
        field_point.header.frame_id = ground_foot
        field_point.header.stamp = stamp
        field_point.pose.position.x = 0.0
        field_point.pose.position.y = 0.0
        field_point.pose.position.z = object_height
        field_point = self.tf_buffer.transform(field_point, "L_CAMERA")

        msg = Marker()
        msg.header.frame_id = "L_CAMERA"
        msg.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
        msg.pose.position = field_normal.pose.position

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.type = Marker.SPHERE
        msg.lifetime = rospy.Duration(100)
        msg.action = msg.ADD
        msg.ns = "ball_finder"
        msg.id = 1
        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.05
        msg.color.r = 0
        msg.color.g = 0
        msg.color.b = 1
        msg.color.a = 1

        self.marker_pub.publish(msg)

        msg.pose.position = field_point.pose.position
        msg.id = 2
        self.marker_pub.publish(msg)

        msg.header.frame_id = "L_CAMERA"
        msg.type = Marker.CUBE
        msg.id = 3
        msg.scale.x = 5
        msg.scale.y = 5
        msg.scale.z = 0.01
        msg.pose.position = field_point.pose.position
        msg.pose.orientation = self.tf_buffer.lookup_transform("L_CAMERA", ground_foot, stamp).transform.rotation

        self.marker_pub.publish(msg)

        msg.header.frame_id = "L_CAMERA"
        msg.type = Marker.SPHERE
        msg.id = 4
        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.05
        msg.pose.position = point_on_image.pose.position

        self.marker_pub.publish(msg)

        """np_field_normal = np.array([field_normal.pose.position.x, field_normal.pose.position.y, field_normal.pose.position.z])
        np_field_point = np.array([field_point.pose.position.x, field_point.pose.position.y, field_point.pose.position.z])
        np_point_on_image = np.array([point_on_image.pose.position.x, point_on_image.pose.position.y, point_on_image.pose.position.z])
        p = self.line_plane_collision(np_field_normal, np_field_point, np_point_on_image, np.array([0, 0, 0]))
        
        #if an intersection can be found...
        if len(p):
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            return point
        else:
            return None"""
        return self.GetIntersectionEG(field_point.pose.position, field_normal.pose.position, point_on_image.pose.position)

    def GetIntersectionEG(self, Ep, normal, Gr, epsilon=1e-6):
        """
        Ep = Ortsvector der Ebene
        Er1 = Richtungsvetor 1 von Ep aus
        Er2 = Richtungsvetor 2 von Ep aus
        Gp = Ortsvektor auf der Geraden
        Gr = Richtungvektor von Gp aus
        """
        t = ((Ep.x * normal.x) + (Ep.y * normal.y) + (Ep.z * normal.z)) / \
            ((normal.x * Gr.x) + (normal.y * Gr.y) + (normal.z * Gr.z))
        print t
        return Point(Gr.x * t, Gr.y * t, Gr.z * t)

    def line_plane_collision(self, planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):

        ndotu = planeNormal.dot(rayDirection)
        if abs(ndotu) < epsilon:
            return []

        w = rayPoint - planePoint
        si = -planeNormal.dot(w) / ndotu
        Psi = w + si * rayDirection + planePoint
        return Psi

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformBall()
