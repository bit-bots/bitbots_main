#!/usr/bin/env python2.7
import rospy
from bitbots_quintic_walk.msg import WalkingDebug
from humanoid_league_msgs.msg import BallRelative, BallsInImage, \
LineInformationInImage, LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative, \
ObstaclesInImage, ObstaclesRelative, ObstacleRelative, \
GoalInImage, GoalRelative, FieldBoundaryInImage, PixelsRelative, \
PixelRelative, GoalPartsInImage, GoalPartsRelative, GoalPostRelative, GoalBarRelative
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


        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # time 0 takes the most current transform available
        self.tf_buffer.can_transform("base_footprint", "camera_optical_frame", rospy.Time(0), timeout=rospy.Duration(30))
        rospy.Subscriber(rospy.get_param("~ball/ball_topic", "/balls_in_image"),
                         BallsInImage,
                         self._callback_ball,
                         queue_size=1)
        if rospy.get_param("~lines/lines_relative", True):
            rospy.Subscriber(rospy.get_param("~lines/lines_topic", "/line_in_image"),
                             LineInformationInImage,
                             self._callback_lines, queue_size=1)
        if rospy.get_param("~lines/pointcloud", False):
            rospy.Subscriber(rospy.get_param("~lines/lines_topic", "/line_in_image"),
                             LineInformationInImage,
                             self._callback_lines_pc, queue_size=1)

        rospy.Subscriber(rospy.get_param("~goals/goals_topic", "/goal_in_image"),
                         GoalInImage, self._callback_goal, queue_size=1)

        rospy.Subscriber(rospy.get_param("~goal_parts/goal_parts_topic", "/goal_parts_in_image"),
                         GoalPartsInImage, self._callback_goal_parts, queue_size=1)

        rospy.Subscriber(rospy.get_param("~obstacles/obstacles_topic", "/obstacles_in_image"),
                         ObstaclesInImage, self._callback_obstacles, queue_size=1)

        rospy.Subscriber(rospy.get_param("~field_boundary/field_boundary_topic", "/field_boundary_in_image"),
                         FieldBoundaryInImage, self._callback_field_boundary, queue_size=1)

        rospy.Subscriber(rospy.get_param("~camera_info/camera_info_topic", "/camera_info"),
                         CameraInfo,
                         self._callback_camera_info,
                         queue_size=1)

        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=1)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
        if rospy.get_param("~lines/lines_relative", True):
            self.line_relative_pub = rospy.Publisher("line_relative", LineInformationRelative, queue_size=1)
        if rospy.get_param("~lines/pointcloud", True):
            self.line_relative_pc_pub = rospy.Publisher("line_relative_pc", PointCloud2, queue_size=1)
        self.goal_relative_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)
        self.goal_parts_relative = rospy.Publisher("goal_parts_relative", GoalPartsRelative, queue_size=1)
        self.obstacle_relative_pub = rospy.Publisher("obstacles_relative", ObstaclesRelative, queue_size=1)
        self.field_boundary_pub = rospy.Publisher("field_boundary_relative", PixelsRelative, queue_size=1)

        self.camera_info = None

        self.ball_height = rospy.get_param("~ball/ball_radius", 0.075)
        self.bar_height = rospy.get_param("~goal_parts/bar_height", 2.0)
        self.publish_frame = "base_footprint"

        rospy.spin()

    def _callback_camera_info(self, camera_info):
        if camera_info.K[0] == 0:
            rospy.logerr_throttle(5.0, "Invalid CameraInfo received. Check your camera settings.")
        self.camera_info = camera_info

    def _callback_ball(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
            return

        field = self.get_plane(msg.header.stamp, self.ball_height, "base_footprint")
        if field is None:
            return

        br = BallRelative()
        br.header.stamp = msg.header.stamp
        br.header.frame_id = self.publish_frame

        # TODO: warning if multiple balls in msg
        for ball in msg.candidates:
            br.ball_relative = self.transform(ball.center, field, msg.header.stamp)
            br.confidence = ball.confidence

            # TODO: This publishes every ball in balls_in_img after each other
            if br.ball_relative is not None:
                self.ball_relative_pub.publish(br)
            else:
                rospy.logwarn("got a ball i could not transform, would be too far away" +
                              " x: " + str(ball.center.x) + " y: " + str(ball.center.y))

    def _callback_lines(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        line = LineInformationRelative()
        line.header.stamp = msg.header.stamp
        line.header.frame_id = self.publish_frame

        for seg in msg.segments:
            rel_seg = LineSegmentRelative()
            rel_seg.start = self.transform(seg.start, field, msg.header.stamp)
            rel_seg.end = self.transform(seg.end, field, msg.header.stamp)

            rel_seg.confidence = seg.confidence

            # only proceed if all transformations were successful
            if rel_seg.start is not None and rel_seg.end is not None:
                line.segments.append(rel_seg)

        for circle in msg.circles:
            rel_circle = LineCircleRelative()
            rel_circle.left = self.transform(circle.left, field, msg.header.stamp)
            rel_circle.middle = self.transform(circle.middle, field, msg.header.stamp)
            rel_circle.right = self.transform(circle.right, field, msg.header.stamp)

            rel_circle.confidence = circle.confidence

            # only proceed if all transformations were successful
            if rel_circle.left is not None and rel_circle.middle is not None and rel_circle.right is not None:
                line.circles.append(rel_circle)

        for intersection in msg.intersections:
            rel_inter = LineIntersectionRelative()
            broken = False
            for segment in intersection.segments:
                rel_seg = LineSegmentRelative()
                rel_seg.start = self.transform(segment.start, field, msg.header.stamp)
                rel_seg.end = self.transform(segment.end, field, msg.header.stamp)

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
            self.warn_camera_info()
            return

        points = []
        field = self.get_plane(msg.header.stamp, 0, "base_footprint")
        if field is None:
            return
        for seg in msg.segments:
            transformed = self.transform(seg.start,field, msg.header.stamp)
            if transformed is not None:
                points.append([transformed.x, transformed.y, transformed.z])
        pc_header = msg.header
        pc_header.frame_id = self.publish_frame
        self.line_relative_pc_pub.publish(pc2.create_cloud_xyz32(pc_header, points))

    def _callback_goal(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        goal = GoalRelative()
        goal.header.stamp = msg.header.stamp
        goal.header.frame_id = self.publish_frame

        transformed_left = self.transform(msg.left_post.foot_point, field, msg.header.stamp)
        if transformed_left is None:
            rospy.logwarn_throttle(5.0,
                                   "Got a left post with foot point ("
                                   + str(msg.left_post.foot_point.x)
                                   + str(msg.left_post.foot_point.y) + ") I could not transform.")
        else:
            goal.left_post = transformed_left



        # Messages do not contain None values so the coordinates have to be checked
        if msg.right_post.foot_point.x != 0 and msg.right_post.foot_point.y != 0:
            transformed_right = self.transform(msg.right_post.foot_point, field, msg.header.stamp)
            if transformed_right is None:
                rospy.logwarn_throttle(5.0,
                                       "Got a left post with foot point ("
                                       + str(msg.left_post.foot_point.x)
                                       + str(msg.left_post.foot_point.y) + ") I could not transform.")
            else:
                goal.right_post = transformed_right

        #TODO evaluate whether we need center direction
        #goal.center_direction.x = goal.left_post.x + (goal.right_post.x - goal.left_post.x) / 2.0
        #goal.center_direction.y = goal.left_post.y + (goal.right_post.y - goal.left_post.y) / 2.0

        goal.confidence = msg.confidence

        self.goal_relative_pub.publish(goal)


    def _callback_goal_parts(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        bar_plane = self.get_plane(msg.header.stamp, self.bar_height, "base_footprint")
        if bar_plane is None:
            return

        # Create new message

        goal_parts_relative_msg = GoalPartsRelative()
        goal_parts_relative_msg.header.stamp = msg.header.stamp
        goal_parts_relative_msg.header.frame_id = self.publish_frame

        # Transform goal posts

        for goal_post_in_image in msg.posts:
            relative_foot_point = self.transform(goal_post_in_image.foot_point, field, msg.header.stamp)
            if relative_foot_point is None:
                rospy.logwarn_throttle(5.0,
                    "Got a post with foot point ({},{}) I could not transform.".format(
                        goal_post_in_image.foot_point.x,
                        goal_post_in_image.foot_point.y,
                    ))
            else:
                post_relative = GoalPostRelative()
                post_relative.foot_point = relative_foot_point
                post_relative.confidence = goal_post_in_image.confidence
                goal_parts_relative_msg.posts.append(post_relative)

        # Transform goal bars

        for goal_bar_in_image in msg.bars:
            relative_left_point = self.transform(goal_bar_in_image.left_point, bar_plane, msg.header.stamp)
            relative_right_point = self.transform(goal_bar_in_image.right_point, bar_plane, msg.header.stamp)
            if relative_right_point is None or relative_left_point is None:
                rospy.logwarn_throttle(5.0,
                    "Got a bar with end points ({},{}) and ({},{}) I could not transform.".format(
                        goal_bar_in_image.left_point.x,
                        goal_bar_in_image.left_point.y,
                        goal_bar_in_image.right_point.x,
                        goal_bar_in_image.right_point.y,
                    ))
            else:
                bar_relative = GoalBarRelative()
                bar_relative.left_point = relative_left_point
                bar_relative.right_point = relative_right_point
                bar_relative.confidence = goal_bar_in_image.confidence
                goal_parts_relative_msg.bars.append(bar_relative)

        self.goal_parts_relative.publish(goal_parts_relative_msg)

    def _callback_obstacles(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
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
            obstacle.position = self.transform(point, field, msg.header.stamp)
            obstacles.obstacles.append(obstacle)

        self.obstacle_relative_pub.publish(obstacles)

    def _callback_field_boundary(self, msg):
        if self.camera_info is None:
            self.warn_camera_info()
            return

        field = self.get_plane(msg.header.stamp, 0.0, "base_footprint")
        if field is None:
            return

        field_boundary = PixelsRelative()
        field_boundary.header = msg.header

        for p in msg.field_boundary_points:
            p_relative = self.transform(p, field, msg.header.stamp)
            if p_relative is not None:
                field_boundary.pixels.append(p_relative)
            else:
                rospy.logwarn("At least one point of the Field Boundary could not be transformed, dropping message")
                return

        self.field_boundary_pub.publish(field_boundary)

    def warn_camera_info(self):
        rospy.logerr_throttle(5.0, "Did not receive CameraInfo.")

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
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, "Could not transform from " + base_frame + " to " + self.camera_info.header.frame_id)
            rospy.logwarn_throttle(5.0, ex)
            return None
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn_throttle(5.0, "Waiting for transforms to become available...")
            rospy.logwarn_throttle(5.0, ex)
            return None

        field_point = PointStamped()
        field_point.header.frame_id = base_frame
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        try:
            field_point = self.tf_buffer.transform(field_point, self.camera_info.header.frame_id)
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, "Could not transform from " + base_frame + " to " + self.camera_info.header.frame_id)
            rospy.logwarn_throttle(5.0, ex)
            return None

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])

        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_normal, field_point

    def transform(self, point, field, stamp):
        K = self.camera_info.K

        x = (point.x - K[2]) / K[0]
        y = (point.y - K[5]) / K[4]
        z = 1.0

        point_on_image = np.array([x, y, z])

        intersection = line_plane_intersection(field[0], field[1], point_on_image)
        if intersection is None:
            return None
        intersection_stamped = PointStamped()
        intersection_stamped.point = intersection
        intersection_stamped.header.stamp = stamp
        intersection_stamped.header.frame_id = self.camera_info.header.frame_id
        try:
            intersection_transformed = self.tf_buffer.transform(intersection_stamped, self.publish_frame)
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, "Could not transform from " + self.publish_frame + " to " + intersection_stamped.header.frame_id)
            rospy.logwarn_throttle(5.0, ex)
            return None
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn_throttle(5.0, "Waiting for transforms to become available...")
            rospy.logwarn_throttle(5.0, ex)
            return None

        return intersection_transformed.point


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
