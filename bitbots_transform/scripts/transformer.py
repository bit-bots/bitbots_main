#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallRelative, BallsInImage, \
LineInformationInImage, LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative, \
ObstaclesInImage, ObstaclesRelative, ObstacleRelative, \
GoalInImage, GoalRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
import tf2_ros
import math
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
import numpy as np


class TransformBall(object):
    def __init__(self):
        rospy.init_node("bitbots_transformer")

        rospy.Subscriber("ball_in_image", BallsInImage, self._callback_ball, queue_size=1)
        rospy.Subscriber("line_in_image", LineInformationInImage, self._callback_lines, queue_size=1)
        #rospy.Subscriber("goal_in_image", GoalInImage, self._callback_goal, queue_size=1)
        #rospy.Subscriber("obstacles_in_image", ObstaclesInImage, self._callback_obstacles, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info, queue_size=1)

        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=10)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=10)
        self.line_relative_pub = rospy.Publisher("line_relative", LineInformationRelative, queue_size=10)
        self.goal_relative_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size=10)
        self.obstacle_relative_pub = rospy.Publisher("obstacles_relative", ObstaclesRelative, queue_size=10)

        self.camera_info = None
        self.focal_length = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_height = 0.1

        rospy.spin()

    def _callback_camera_info(self, camerainfo):
        self.camera_info = camerainfo
        self.focal_length = float(camerainfo.K[0])/ (camerainfo.width / 2.0)

    def _callback_ball(self, msg):
        if self.camera_info is None:
            rospy.logerr_throttle(rospy.Rate(1.0), "did not receive camerainfo")

        field = self.get_plane(msg.header.stamp, self.ball_height)
        if field is None:
            return

        br = BallRelative()
        br.header.stamp = msg.header.stamp
        br.header.frame_id = "L_CAMERA"

        for ball in msg.candidates:
            br.ball_relative = self.transform(ball.center, field)
            br.confidence = ball.confidence

            if br.ball_relative is not None:
                self.ball_relative_pub.publish(br)
            else:
                rospy.logwarn("got a ball i could not transform, would be too far away" +
                              " x: " + ball.center.x + " y: " + ball.center.y)

    def _callback_lines(self, msg):
        if self.camera_info is None:
            rospy.logerr_throttle(rospy.Rate(1.0), "did not receive camerainfo")

        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        line = LineInformationRelative()
        line.header.stamp = msg.header.stamp
        line.header.frame_id = "L_CAMERA"

        for seg in msg.segments:
            rel_seg = LineSegmentRelative()
            rel_seg.start = self.transform(seg.start, field)
            rel_seg.end = self.transform(seg.end, field)

            rel_seg.confidence = seg.confidence

            # only proceed if all transformations were successful
            if rel_seg.start is not None and rel_seg.end is not None:
                line.segments.append(rel_seg)
            else:
                rospy.logwarn("got a segment i could not transform")

        for circle in msg.circles:
            rel_circle = LineCircleRelative()
            rel_circle.left = self.transform(circle.left,field)
            rel_circle.middle = self.transform(circle.middle, field)
            rel_circle.right = self.transform(circle.right, field)

            rel_circle.confidence = circle.confidence

            # only proceed if all transformations were successful
            if rel_circle.left is not None and rel_circle.middle is not None and rel_circle.right is not None:
                line.circles.append(rel_circle)

        for intersection in msg.intersections:
            rel_inter = LineIntersectionRelative()
            broken = False
            for segment in intersection.segments:
                rel_seg = LineSegmentRelative()
                rel_seg.start = self.transform(segment.start, field)
                rel_seg.end = self.transform(segment.end, field)

                rel_seg.confidence = segment.confidence

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

    def _callback_goal(self, msg):
        gr = GoalRelative()
        gr.header.stamp = msg.header.stamp
        gr.header.frame_id = "L_CAMERA"
        return


    def _callback_obstacles(self, msg):
        return

    def get_plane(self, stamp, object_height):
        """ returns a plane which an object is believed to be on as a tuple of a point on this plane and a normal"""
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
        try:
            field_normal = self.tf_buffer.transform(field_normal, "L_CAMERA")
        except tf2_ros.LookupException:
            rospy.logwarn_throttle(rospy.Rate(1.0), "Could not transform from " + ground_foot + " to L_CAMERA")
            return None

        field_point = PointStamped()
        field_point.header.frame_id = ground_foot
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        try:
            field_point = self.tf_buffer.transform(field_point, "L_CAMERA")
        except tf2_ros.LookupException:
            rospy.logwarn_throttle(rospy.Rate(1.0), "Could not transform from " + ground_foot + " to L_CAMERA")
            return None

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])

        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_normal, field_point

    def transform(self, point, field):

        # normalized camera coordinates coordinate system
        #  -1,0 ------------- 1,1
        #       |           |
        #       |    0,0    |
        #       |           |
        # -1,-1 ------------- 1,0
        normalized_x = 2 * (float(point.x) - (float(self.camera_info.width) / 2)) / float(self.camera_info.width)
        normalized_y = (-2 * (float(point.y) - (float(self.camera_info.height) / 2)) / float(self.camera_info.height)) * \
                       (float(self.camera_info.height) / float(self.camera_info.width))

        point_on_image = np.array([self.focal_length, -normalized_x, normalized_y])

        return line_plane_intersection(field[0], field[1], point_on_image)


def line_plane_intersection(plane_normal, plane_point, ray_direction, epsilon=1e-3):
    ndotu = plane_normal.dot(ray_direction)
    # checking if intersection can be found at reasonable distance or at all
    if abs(ndotu) < epsilon:
        return None

    si = -plane_normal.dot(- plane_point) / ndotu

    # we are casting a ray, intersections need to be in front of the camera
    if si < 0:
        return None

    intersection = si * ray_direction
    return Point(intersection[0], intersection[1], intersection[2])


if __name__ == "__main__":
    TransformBall()
