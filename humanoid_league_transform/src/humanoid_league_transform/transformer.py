#!/usr/bin/env python2.7
import rospy
from bitbots_quintic_walk.msg import WalkingDebug
from humanoid_league_msgs.msg import BallRelative, BallsInImage, \
LineInformationInImage, LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative, \
ObstaclesInImage, ObstaclesRelative, ObstacleRelative, \
GoalInImage, GoalRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import math
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
import numpy as np


class TransformBall(object):
    def __init__(self):
        rospy.init_node("bitbots_transformer")

        rospy.Subscriber(rospy.get_param("transformer/ball/ball_topic", "ball_in_image"),
                         BallsInImage,
                         self._callback_ball,
                         queue_size=1)
        if rospy.get_param("transformer/lines/lines_relative", True):
            rospy.Subscriber(rospy.get_param("transformer/lines/lines_topic", "line_in_image"),
                             LineInformationInImage,
                             self._callback_lines, queue_size=1)
        if rospy.get_param("transformer/lines/pointcloud", False):
            rospy.Subscriber(rospy.get_param("transformer/lines/lines_topic", "line_in_image"),
                             LineInformationInImage,
                             self._callback_lines_pc, queue_size=1)

        rospy.Subscriber(rospy.get_param("transformer/goals/goals_topic", "goal_in_image"),
                         GoalInImage, self._callback_goal, queue_size=1)
        rospy.Subscriber(rospy.get_param("transformer/obstacles/obstacles_topic", "obstacles_in_image"),
                         ObstaclesInImage, self._callback_obstacles, queue_size=1)

        rospy.Subscriber(rospy.get_param("transformer/camera_info/camera_info_topic", "camera_info"),
                         CameraInfo,
                         self._callback_camera_info,
                         queue_size=1)

        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=1)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
        if rospy.get_param("transformer/lines/lines_relative", True):
            self.line_relative_pub = rospy.Publisher("line_relative", LineInformationRelative, queue_size=1)
        if rospy.get_param("transformer/lines/pointcloud", False):
            self.line_relative_pc_pub = rospy.Publisher("line_relative_pc", PointCloud2, queue_size=1)
        self.goal_relative_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)
        self.obstacle_relative_pub = rospy.Publisher("obstacles_relative", ObstaclesRelative, queue_size=1)

        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_height = rospy.get_param("transformer/ball/ball_radius", 0.075)
        self.publish_frame = "camera_optical_frame"


        rospy.spin()

    def _callback_camera_info(self, camera_info):
        self.camera_info = camera_info

    def _callback_ball(self, msg):
        if self.camera_info is None:
            rospy.logerr_throttle(rospy.Rate(1.0), "did not receive camerainfo")

        field = self.get_plane(msg.header.stamp, self.ball_height, "base_footprint")
        if field is None:
            return

        br = BallRelative()
        br.header.stamp = msg.header.stamp
        br.header.frame_id = self.publish_frame

        for ball in msg.candidates:
            br.ball_relative = self.transform(ball.center, field)
            br.confidence = ball.confidence

            if br.ball_relative is not None:
                self.ball_relative_pub.publish(br)
            else:
                rospy.logwarn("got a ball i could not transform, would be too far away" +
                              " x: " + str(ball.center.x) + " y: " + str(ball.center.y))

    def _callback_lines(self, msg):
        if self.camera_info is None:
            rospy.logerr( "did not receive camerainfo")

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        line = LineInformationRelative()
        line.header.stamp = msg.header.stamp
        line.header.frame_id = self.publish_frame

        for seg in msg.segments:
            rel_seg = LineSegmentRelative()
            rel_seg.start = self.transform(seg.start, field)
            rel_seg.end = self.transform(seg.end, field)

            rel_seg.confidence = seg.confidence

            # only proceed if all transformations were successful
            if rel_seg.start is not None and rel_seg.end is not None:
                line.segments.append(rel_seg)

            else:
                rospy.logwarn_throttle(1.0, "got a segment i could not transform")

        for circle in msg.circles:
            rel_circle = LineCircleRelative()
            rel_circle.left = self.transform(circle.left, field)
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

    def _callback_lines_pc(self, msg):
        if self.camera_info is None:
            rospy.logerr("did not receive camerainfo")
            return
        points = []
        field = self.get_plane(msg.header.stamp, 0, "base_footprint")
        if field is None:
            return
        for seg in msg.segments:
            transformed = self.transform(seg.start,field)
            if transformed is not None:
                points.append([transformed.x, transformed.y, transformed.z])
        pc_header = msg.header
        pc_header.frame_id = self.publish_frame
        self.line_relative_pc_pub.publish(pc2.create_cloud_xyz32(pc_header, points))

    def _callback_goal(self, msg):
        if self.camera_info is None:
            rospy.logerr("did not receive camerainfo")
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        goal = GoalRelative()
        goal.header.stamp = msg.header.stamp
        goal.header.frame_id = self.publish_frame

        transformed_left = self.transform(msg.left_post.foot_point, field)
        goal.left_post = transformed_left

        if msg.right_post.foot_point.x != 0:
            transformed_right = self.transform(msg.right_post.foot_point, field)
            goal.right_post = transformed_right

        goal.confidence = msg.confidence

        self.goal_relative_pub.publish(goal)

    def _callback_obstacles(self, msg):
        if self.camera_info is None:
            rospy.logerr("did not receive camerainfo")
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        obstacles = ObstaclesRelative()
        obstacles.header = msg.header
        obstacles.header.frame_id = self.publish_frame

        for o in msg.obstacles:
            obstacle = ObstacleRelative()
            obstacle.playerNumber = o.playerNumber
            obstacle.confidence = o.confidence
            obstacle.color = o.color
            point = Point()
            point.x = o.top_left.x + o.height
            point.y = o.top_left.y + o.width/2
            obstacle.position = self.transform(point, field)
            obstacles.obstacles.append(obstacle)

        self.obstacle_relative_pub.publish(obstacles)

    def get_plane(self, stamp, object_height, base_frame):
        """ returns a plane which an object is believed to be on as a tuple of a point on this plane and a normal"""
        field_normal = PointStamped()
        field_normal.header.frame_id = base_frame
        field_normal.header.stamp = stamp
        field_normal.point.x = 0.0
        field_normal.point.y = 0.0
        field_normal.point.z = 1.0
        try:
            field_normal = self.tf_buffer.transform(field_normal, self.camera_info.header.frame_id, timeout=rospy.Duration(0.2))
        except tf2_ros.LookupException:
            rospy.logwarn("Could not transform from " + base_frame + " to " + self.camera_info.header.frame_id)
            return None
        except tf2_ros.ExtrapolationException:
            rospy.logwarn("Waiting for transforms to become available...")
            return None

        field_point = PointStamped()
        field_point.header.frame_id = base_frame
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        try:
            field_point = self.tf_buffer.transform(field_point, self.camera_info.header.frame_id)
        except tf2_ros.LookupException:
            rospy.logwarn("Could not transform from " + base_frame + " to " + self.camera_info.header.frame_id)
            return None

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])

        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_normal, field_point

    def transform(self, point, field):
        K = self.camera_info.K
        
        x = (point.x - K[2]) / K[0]
        y = (point.y - K[5]) / K[4]
        z = 1.0

        point_on_image = np.array([x, y, z])

        return line_plane_intersection(field[0], field[1], point_on_image)


def line_plane_intersection(plane_normal, plane_point, ray_direction):
    ndotu = plane_normal.dot(ray_direction)

    si = -plane_normal.dot(- plane_point) / ndotu

    # we are casting a ray, intersections need to be in front of the camera
    if si < 0:
        return None

    intersection = si * ray_direction
    return Point(intersection[0], intersection[1], intersection[2])


if __name__ == "__main__":
    TransformBall()
