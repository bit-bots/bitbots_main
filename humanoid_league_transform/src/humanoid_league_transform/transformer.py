#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallRelative, BallsInImage, \
    LineInformationInImage, \
    LineInformationRelative, LineSegmentRelative, LineCircleRelative, LineIntersectionRelative, \
    ObstaclesInImage, ObstaclesRelative, ObstacleRelative, \
    GoalInImage, GoalRelative, GoalPartsInImage, GoalPartsRelative, GoalPostRelative, GoalBarRelative, \
    FieldBoundaryInImage, PixelsRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from tf2_geometry_msgs import PointStamped
import numpy as np


class TransformBall(object):
    def __init__(self):
        rospy.init_node("humanoid_league_transformer")

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Parameters
        self._ball_height = rospy.get_param("~ball/ball_radius", 0.075)
        self._bar_height = rospy.get_param("~goal_parts/bar_height", 2.0)
        self._publish_frame = rospy.get_param("~publish_frame", "base_footprint")

        camera_info_topic = rospy.get_param("~camera_info/camera_info_topic", "/camera_info")
        balls_in_image_topic = rospy.get_param("~ball/ball_topic", "/balls_in_image")
        lines_in_image_topic = rospy.get_param("~lines/lines_topic", "/line_in_image")
        goal_in_image_topic = rospy.get_param("~goals/goals_topic", "/goal_in_image")
        goal_parts_in_image_topic = rospy.get_param("~goal_parts/goal_parts_topic", "/goal_parts_in_image")
        obstacles_in_image_topic = rospy.get_param("~obstacles/obstacles_topic", "/obstacles_in_image")
        field_boundary_in_image_topic = rospy.get_param("~field_boundary/field_boundary_topic",
                                                        "/field_boundary_in_image")

        publish_lines_as_lines_relative = rospy.get_param("~lines/lines_relative", True)
        publish_lines_as_pointcloud = rospy.get_param("~lines/pointcloud", False)

        self._camera_info = None
        rospy.Subscriber(camera_info_topic, CameraInfo, self._callback_camera_info, queue_size=1)

        # Wait for Camera info
        cam_info_counter = 0
        while self._camera_info is None:
            rospy.sleep(0.1)
            if cam_info_counter > 50:
                rospy.logerr_throttle(5, rospy.get_name() + ": Camera Info not received on topic '" +
                                      camera_info_topic + "'")

        # Wait up to 5 seconds for transforms to become available, then print an error and try again
        # rospy.Time(0) gets the most recent transform
        while not self._tf_buffer.can_transform(self._publish_frame,
                                                self._camera_info.header.frame_id,
                                                rospy.Time(0),
                                                timeout=rospy.Duration(5)):
            rospy.logerr(rospy.get_name() + ": Could not get transformation from " + self._publish_frame +
                         "to " + self._camera_info.header.frame_id)

        # Also check if we can transform from optical frame to base_footprint
        while not self._tf_buffer.can_transform("base_footprint",
                                                self._camera_info.header.frame_id,
                                                rospy.Time(0),
                                                timeout=rospy.Duration(5)):
            rospy.logerr(rospy.get_name() + ": Could not get transformation from 'base_footprint' to " +
                         self._camera_info.header.frame_id)

        # Publishers TODO make topics configurable
        self._ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
        if publish_lines_as_lines_relative:
            self._line_relative_pub = rospy.Publisher("line_relative", LineInformationRelative, queue_size=1)
        if publish_lines_as_pointcloud:
            self._line_relative_pc_pub = rospy.Publisher("line_relative_pc", PointCloud2, queue_size=1)
        self._goal_relative_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)
        self._goal_parts_relative = rospy.Publisher("goal_parts_relative", GoalPartsRelative, queue_size=1)
        self._obstacle_relative_pub = rospy.Publisher("obstacles_relative", ObstaclesRelative, queue_size=1)
        self._field_boundary_pub = rospy.Publisher("field_boundary_relative", PixelsRelative, queue_size=1)

        # Subscribers
        rospy.Subscriber(balls_in_image_topic, BallsInImage, self._callback_ball, queue_size=1)
        if publish_lines_as_lines_relative:
            rospy.Subscriber(lines_in_image_topic, LineInformationInImage, self._callback_lines, queue_size=1)
        if publish_lines_as_pointcloud:
            rospy.Subscriber(lines_in_image_topic, LineInformationInImage, self._callback_lines_pc, queue_size=1)
        rospy.Subscriber(goal_in_image_topic, GoalInImage, self._callback_goal, queue_size=1)
        rospy.Subscriber(goal_parts_in_image_topic,  GoalPartsInImage, self._callback_goal_parts, queue_size=1)
        rospy.Subscriber(obstacles_in_image_topic, ObstaclesInImage, self._callback_obstacles, queue_size=1)
        rospy.Subscriber(field_boundary_in_image_topic, FieldBoundaryInImage,
                         self._callback_field_boundary, queue_size=1)

        rospy.spin()

    def _callback_camera_info(self, camera_info):
        if camera_info.K[0] == 0:
            rospy.logerr_throttle(5.0, rospy.get_name() + ": Invalid CameraInfo received. Check your camera settings.")
        self._camera_info = camera_info

    def _callback_ball(self, msg):
        field = self.get_plane(msg.header.stamp, self._ball_height)
        if field is None:
            return

        br = BallRelative()
        br.header.stamp = msg.header.stamp
        br.header.frame_id = self._publish_frame

        if len(msg.candidates) > 1:
            rospy.logwarn_throttle(5.0, "Multiple ball candidates in BallsInImage message. " +
                                   "They will be published as multiple BallRelative messages")
        for ball in msg.candidates:
            br.ball_relative = self._transform(ball.center, field, msg.header.stamp)
            br.confidence = ball.confidence

            if br.ball_relative is not None:
                self._ball_relative_pub.publish(br)
            else:
                rospy.logwarn_throttle(5.0, rospy.get_name() + ": Got a ball I could not transform")

    def _callback_lines(self, msg):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        line = LineInformationRelative()
        line.header.stamp = msg.header.stamp
        line.header.frame_id = self._publish_frame

        for seg in msg.segments:
            rel_seg = LineSegmentRelative()
            rel_seg.start = self._transform(seg.start, field, msg.header.stamp)
            rel_seg.end = self._transform(seg.end, field, msg.header.stamp)

            rel_seg.confidence = seg.confidence

            # only proceed if all transformations were successful
            if rel_seg.start is not None and rel_seg.end is not None:
                line.segments.append(rel_seg)

        for circle in msg.circles:
            rel_circle = LineCircleRelative()
            rel_circle.left = self._transform(circle.left, field, msg.header.stamp)
            rel_circle.middle = self._transform(circle.middle, field, msg.header.stamp)
            rel_circle.right = self._transform(circle.right, field, msg.header.stamp)

            rel_circle.confidence = circle.confidence

            # only proceed if all transformations were successful
            if rel_circle.left is not None and rel_circle.middle is not None and rel_circle.right is not None:
                line.circles.append(rel_circle)

        for intersection in msg.intersections:
            rel_inter = LineIntersectionRelative()
            broken = False
            for segment in intersection.segments:
                rel_seg = LineSegmentRelative()
                rel_seg.start = self._transform(segment.start, field, msg.header.stamp)
                rel_seg.end = self._transform(segment.end, field, msg.header.stamp)

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
        if line.segments or line.circles or line.intersections:
            self._line_relative_pub.publish(line)
        else:
            rospy.logwarn_throttle(5.0, rospy.get_name() +
                                   ": Could not transform any segments, circles or intersections" +
                                   " in LineInformationInImage message.")

    def _callback_lines_pc(self, msg):
        field = self.get_plane(msg.header.stamp, 0)
        if field is None:
            return

        points = np.zeros((len(msg.segments), 3))
        num_transformed_correctly = 0
        for i in range(len(msg.segments)):
            transformed = self._transform(msg.segments[i].start, field, msg.header.stamp)
            if transformed is not None:
                points[i]([transformed.x, transformed.y, transformed.z])
                num_transformed_correctly += 1

        if num_transformed_correctly == 0:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": No line points could be transformed")
        pc_header = msg.header
        pc_header.frame_id = self._publish_frame
        self._line_relative_pc_pub.publish(pc2.create_cloud_xyz32(pc_header, points[:num_transformed_correctly]))

    def _callback_goal(self, msg):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        goal = GoalRelative()
        goal.header.stamp = msg.header.stamp
        goal.header.frame_id = self._publish_frame

        transformed_left = self._transform(msg.left_post.foot_point, field, msg.header.stamp)
        if transformed_left is None:
                rospy.logwarn_throttle(5.0, rospy.get_name() +
                                       ": Got a left post with foot point ({},{}) I could not transform.".format(
                                           msg.left_post.foot_point.x,
                                           msg.left_post.foot_point.y))
        else:
            goal.left_post = transformed_left

        # Messages do not contain None values so the coordinates have to be checked
        if msg.right_post.foot_point.x != 0 and msg.right_post.foot_point.y != 0:
            transformed_right = self._transform(msg.right_post.foot_point, field, msg.header.stamp)
            if transformed_right is None:
                rospy.logwarn_throttle(5.0, rospy.get_name() +
                                       ": Got a right post with foot point ({},{}) I could not transform.".format(
                                           msg.left_post.foot_point.x,
                                           msg.left_post.foot_point.y))
            else:
                goal.right_post = transformed_right

        goal.confidence = msg.confidence
        self._goal_relative_pub.publish(goal)

    def _callback_goal_parts(self, msg):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        bar_plane = self.get_plane(msg.header.stamp, self._bar_height)
        if bar_plane is None:
            return

        # Create new message

        goal_parts_relative_msg = GoalPartsRelative()
        goal_parts_relative_msg.header.stamp = msg.header.stamp
        goal_parts_relative_msg.header.frame_id = self._publish_frame

        # Transform goal posts
        for goal_post_in_image in msg.posts:
            relative_foot_point = self._transform(goal_post_in_image.foot_point, field, msg.header.stamp)
            if relative_foot_point is None:
                rospy.logwarn_throttle(5.0, rospy.get_name() +
                                       ": Got a post with foot point ({},{}) I could not transform.".format(
                                           goal_post_in_image.foot_point.x,
                                           goal_post_in_image.foot_point.y))
            else:
                post_relative = GoalPostRelative()
                post_relative.foot_point = relative_foot_point
                post_relative.confidence = goal_post_in_image.confidence
                goal_parts_relative_msg.posts.append(post_relative)

        # Transform goal bars
        for goal_bar_in_image in msg.bars:
            relative_left_point = self._transform(goal_bar_in_image.left_point, bar_plane, msg.header.stamp)
            relative_right_point = self._transform(goal_bar_in_image.right_point, bar_plane, msg.header.stamp)
            if relative_right_point is None or relative_left_point is None:
                rospy.logwarn_throttle(5.0, rospy.get_name() +
                                       ": Got a bar with end points ({},{}) and ({},{}) I could not transform.".format(
                                           goal_bar_in_image.left_point.x,
                                           goal_bar_in_image.left_point.y,
                                           goal_bar_in_image.right_point.x,
                                           goal_bar_in_image.right_point.y))
            else:
                bar_relative = GoalBarRelative()
                bar_relative.left_point = relative_left_point
                bar_relative.right_point = relative_right_point
                bar_relative.confidence = goal_bar_in_image.confidence
                goal_parts_relative_msg.bars.append(bar_relative)

        self._goal_parts_relative.publish(goal_parts_relative_msg)

    def _callback_obstacles(self, msg):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        obstacles = ObstaclesRelative()
        obstacles.header = msg.header
        obstacles.header.frame_id = self._publish_frame

        for o in msg.obstacles:
            obstacle = ObstacleRelative()
            obstacle.playerNumber = o.playerNumber
            obstacle.confidence = o.confidence
            obstacle.color = o.color
            point = Point()
            point.x = o.top_left.x + o.height
            point.y = o.top_left.y + o.width/2
            position = self._transform(point, field, msg.header.stamp)
            if obstacles.position is not None:
                obstacle.position = position
                obstacles.obstacles.append(obstacle)
            else:
                rospy.logwarn_throttle(5.0, rospy.get_name() + ": Got an obstacle I could not transform")

        self._obstacle_relative_pub.publish(obstacles)

    def _callback_field_boundary(self, msg):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        field_boundary = PixelsRelative()
        field_boundary.header = msg.header

        for p in msg.field_boundary_points:
            p_relative = self._transform(p, field, msg.header.stamp)
            if p_relative is not None:
                field_boundary.pixels.append(p_relative)
            else:
                rospy.logwarn_throttle(5.0, rospy.get_name() +
                                       ": At least one point of the Field Boundary could not be transformed," +
                                       " dropping message")
                return

        self._field_boundary_pub.publish(field_boundary)

    def get_plane(self, stamp, object_height):
        """ returns a plane which an object is believed to be on as a tuple of a point on this plane and a normal"""

        base_frame = "base_footprint"

        field_normal = PointStamped()
        field_normal.header.frame_id = base_frame
        field_normal.header.stamp = stamp
        field_normal.point.x = 0.0
        field_normal.point.y = 0.0
        field_normal.point.z = 1.0
        try:
            field_normal = self._tf_buffer.transform(field_normal,
                                                     self._camera_info.header.frame_id,
                                                     timeout=rospy.Duration(0.2))
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None

        field_point = PointStamped()
        field_point.header.frame_id = base_frame
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        try:
            field_point = self._tf_buffer.transform(field_point, self._camera_info.header.frame_id)
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])

        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_normal, field_point

    def _transform(self, point, field, stamp):
        camera_projection_matrix = self._camera_info.K

        # calculate a point on a projection plane 1 m (for convenience) away
        # (point - image center) / focal length
        x = (point.x - camera_projection_matrix[2]) / camera_projection_matrix[0]
        y = (point.y - camera_projection_matrix[5]) / camera_projection_matrix[4]
        z = 1.0
        point_on_image = np.array([x, y, z])

        intersection = self._line_plane_intersection(field[0], field[1], point_on_image)
        if intersection is None:
            return None

        intersection_stamped = PointStamped()
        intersection_stamped.point = intersection
        intersection_stamped.header.stamp = stamp
        intersection_stamped.header.frame_id = self._camera_info.header.frame_id
        try:
            intersection_transformed = self._tf_buffer.transform(intersection_stamped, self._publish_frame)
        except tf2_ros.LookupException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn_throttle(5.0, rospy.get_name() + ": " + str(ex))
            return None

        return intersection_transformed.point

    def _line_plane_intersection(self, plane_normal, plane_point, ray_direction):
        n_dot_u = plane_normal.dot(ray_direction)
        relative_ray_distance = -plane_normal.dot(- plane_point) / n_dot_u

        # we are casting a ray, intersections need to be in front of the camera
        if relative_ray_distance < 0:
            return None

        intersection = relative_ray_distance * ray_direction
        return Point(intersection[0], intersection[1], intersection[2])


if __name__ == "__main__":
    TransformBall()
