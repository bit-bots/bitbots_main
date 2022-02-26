#!/usr/bin/env python3
import cv2
import rclpy
import tf2_ros
import numpy as np
#import sensor_msgs.point_cloud2 as pc2 #TODO
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.publisher import Publisher
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PolygonStamped
from tf2_geometry_msgs import PointStamped
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from soccer_vision_msgs.msg import BallArray, FieldBoundary, GoalpostArray, RobotArray
from humanoid_league_msgs.msg import PoseWithCertaintyArray, PoseWithCertainty, ObstacleRelativeArray, ObstacleRelative

class Transformer(Node):
    def __init__(self):
        super().__init__('humanoid_league_transform')

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cv_bridge = CvBridge()

        self.declare_parameter('ball.ball_radius', 0.0)
        self.declare_parameter('goalposts.bar_height', 0.0)
        self.declare_parameter('publish_frame', "")
        self.declare_parameter('base_footprint_frame', "")
        self.declare_parameter('obstacles.footpoint_out_of_image_threshold', 0.0)
        self.declare_parameter('goalposts.footpoint_out_of_image_threshold', 0.0)
        self.declare_parameter('camera_info.camera_info_topic', "")
        self.declare_parameter('ball.ball_topic', "")
        self.declare_parameter('goalposts.goalposts_topic', "")
        self.declare_parameter('obstacles.obstacles_topic', "")
        self.declare_parameter('field_boundary.field_boundary_topic', "")
        self.declare_parameter('masks.line_mask.topic', "")
        self.declare_parameter('masks.line_mask.scale', 0.0)


        # Parameters
        self._ball_height = self.get_parameter("ball.ball_radius").get_parameter_value().double_value
        self._bar_height = self.get_parameter("goalposts.bar_height").get_parameter_value().double_value
        self._publish_frame = self.get_parameter("publish_frame").get_parameter_value().string_value
        self._base_footprint_frame = self.get_parameter("base_footprint_frame").get_parameter_value().string_value
        self._obstacle_footpoint_out_of_image_threshold = \
            self.get_parameter("obstacles.footpoint_out_of_image_threshold").get_parameter_value().double_value
        self._goalpost_footpoint_out_of_image_threshold = \
            self.get_parameter("goalposts.footpoint_out_of_image_threshold").get_parameter_value().double_value
        camera_info_topic = self.get_parameter("camera_info.camera_info_topic").get_parameter_value().string_value
        ball_in_image_array_topic = self.get_parameter("ball.ball_topic").get_parameter_value().string_value
        goalposts_in_image_topic = self.get_parameter("goalposts.goalposts_topic").get_parameter_value().string_value
        obstacles_in_image_topic = self.get_parameter("obstacles.obstacles_topic").get_parameter_value().string_value
        field_boundary_in_image_topic = self.get_parameter("field_boundary.field_boundary_topic").get_parameter_value().string_value
        line_mask_in_image_topic = self.get_parameter("masks.line_mask.topic").get_parameter_value().string_value
        line_mask_scaling = self.get_parameter("masks.line_mask.scale").get_parameter_value().double_value

        self._camera_info = None
        self.create_subscription(CameraInfo, camera_info_topic, self._callback_camera_info, 1)

        # Wait for Camera info
        cam_info_counter = 0
        while self._camera_info is None:
            self.get_clock().sleep_for(Duration(seconds=0.1))
            cam_info_counter += 1
            if cam_info_counter > 100:
                self.get_logger().error(
                    ": Camera Info not received on topic " + camera_info_topic + "",
                    throttle_duration_sec=5)
            if not rclpy.ok():
                return

        # Wait up to 5 seconds for transforms to become available, then print an error and try again
        # Time(0) gets the most recent transform
        while not self._tf_buffer.can_transform(self._publish_frame,
                                                self._camera_info.header.frame_id,
                                                Time(0),
                                                timeout=Duration(seconds=5)):
            self.get_logger().error("Could not get transformation from " + self._publish_frame +
                         "to " + self._camera_info.header.frame_id)

        # Also check if we can transform from optical frame to base_footprint
        while not self._tf_buffer.can_transform(self._base_footprint_frame,
                                                self._camera_info.header.frame_id,
                                                Time(0),
                                                timeout=Duration(seconds=5)):
            self.get_logger().error("Could not get transformation from " + self._base_footprint_frame +
                         " to " + self._camera_info.header.frame_id)

        # Publishers TODO make topics configurable
        self._balls_relative_pub = self.create_publisher(PoseWithCertaintyArray, "balls_relative", 1)
        self._line_mask_relative_pc_pub = self.create_publisher(PointCloud2, "line_mask_relative_pc", 1)
        self._goalposts_relative = self.create_publisher(PoseWithCertaintyArray, "goal_posts_relative", 1)
        self._obstacle_relative_pub = self.create_publisher(ObstacleRelativeArray, "obstacles_relative", 1)
        self._field_boundary_pub = self.create_publisher(PolygonStamped, "field_boundary_relative", 1)

        # Subscribers
        self.create_subscription(BallArray, ball_in_image_array_topic, self._callback_ball, 1)
        self.create_subscription(GoalpostArray, goalposts_in_image_topic, self._callback_goalposts, 1)
        self.create_subscription(RobotArray, obstacles_in_image_topic, self._callback_obstacles, 1)
        self.create_subscription(PolygonStamped, field_boundary_in_image_topic,
                         self._callback_field_boundary, 1)
        self.create_subscription(Image, line_mask_in_image_topic,
            lambda msg: self._callback_masks(
                msg,
                self._line_mask_relative_pc_pub,
                scale=line_mask_scaling), 1)

    def _callback_camera_info(self, camera_info: CameraInfo):
        if camera_info.K[0] == 0:
            self.get_logger().error(
                "Invalid CameraInfo received. Check your camera settings.",
                throttle_duration_sec=5)
        self._camera_info = camera_info

    def _callback_ball(self, msg: BallArray):
        field = self.get_plane(msg.header.stamp, self._ball_height)
        if field is None:
            return

        balls = []
        for ball in msg.balls:
            transformed_ball = self._transform_point(ball.center, field, msg.header.stamp)
            if transformed_ball is not None:
                ball_relative = PoseWithCertainty()
                ball_relative.pose.pose.position = transformed_ball
                ball_relative.confidence = ball.confidence
                balls.append(ball_relative)

        balls_relative = PoseWithCertaintyArray()
        balls_relative.header.stamp = msg.header.stamp
        balls_relative.header.frame_id = self._publish_frame
        balls_relative.poses = balls

        self._balls_relative_pub.publish(balls_relative)

    def _callback_goalposts(self, msg: GoalpostArray):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        # Create new message
        goalposts_relative_msg = PoseWithCertaintyArray()
        goalposts_relative_msg.header.stamp = msg.header.stamp
        goalposts_relative_msg.header.frame_id = self._publish_frame

        # Transform goal posts
        for goal_post_in_image in msg.posts:
            # Check if post is not going out of the image at the bottom
            if not self._object_at_bottom_of_image(
                    goal_post_in_image.bottom.y, self._goalpost_footpoint_out_of_image_threshold):
                # Transform footpoint
                relative_foot_point = self._transform_point(goal_post_in_image.bottom, field, msg.header.stamp)
                if relative_foot_point is None:
                    self.get_logger().warn(
                        "Got a post with foot point ({},{}) I could not transform.".format(
                            goal_post_in_image.bottom.x,
                            goal_post_in_image.bottom.y),
                        throttle_duration_sec=5)
                else:
                    post_relative = PoseWithCertainty()
                    post_relative.pose.pose.position = relative_foot_point
                    post_relative.confidence = goal_post_in_image.confidence
                    goalposts_relative_msg.poses.append(post_relative)

        self._goalposts_relative.publish(goalposts_relative_msg)

    def _callback_obstacles(self, msg: RobotArray):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        obstacles = ObstacleRelativeArray()
        obstacles.header = msg.header
        obstacles.header.frame_id = self._publish_frame

        for robot in msg.robots:
            obstacle = ObstacleRelative()
            obstacle.playerNumber = robot.player_number
            obstacle.pose.confidence = robot.confidence
            obstacle.type = robot.team
            point = Point()
            point.x = robot.center.x
            point.y = robot.center.y + robot.size_y // 2

            # Check if obstacle is not going out of the image at the bottom
            if not self._object_at_bottom_of_image(
                    point.y, self._obstacle_footpoint_out_of_image_threshold):
                position = self._transform_point(point, field, msg.header.stamp)
                if position is not None:
                    obstacle.pose.pose.pose.position = position
                    obstacles.obstacles.append(obstacle)
                else:
                    self.get_logger().warn("Got an obstacle I could not transform", throttle_duration_sec=5)

        self._obstacle_relative_pub.publish(obstacles)

    def _callback_field_boundary(self, msg: FieldBoundary):
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        field_boundary = PolygonStamped()
        field_boundary.header = msg.header
        field_boundary.header.frame_id = self._publish_frame

        for p in msg.points:
            p_relative = self._transform_point(p, field, msg.header.stamp)
            if p_relative is not None:
                field_boundary.polygon.points.append(p_relative)
            else:
                self.get_logger().warn(
                    "At least one point of the Field Boundary could not be transformed, dropping message...",
                    throttle_duration_sec=5)
                return

        self._field_boundary_pub.publish(field_boundary)

    def _callback_masks(self, msg: Image, publisher: Publisher, encoding='8UC1', scale: float = 1.0):
        """
        Projects a mask from the input image as a pointcloud on the field plane.
        """
        # Get field plane
        field = self.get_plane(msg.header.stamp, 0.0)
        if field is None:
            return

        # Convert subsampled image
        image = cv2.resize(
            self._cv_bridge.imgmsg_to_cv2(msg, encoding),
            (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

        # Get indices for all non 0 pixels (the pixels which should be displayed in the pointcloud)
        point_idx_tuple = np.where(image != 0)

        # Restructure index tuple to a array
        point_idx_array = np.empty((point_idx_tuple[0].shape[0], 3))
        point_idx_array[:, 0] = point_idx_tuple[1]
        point_idx_array[:, 1] = point_idx_tuple[0]

        # Project the pixels onto the field plane
        points_on_plane_from_cam = self._get_field_intersection_for_pixels(
            point_idx_array,
            field,
            scale=scale)

        # Make a pointcloud2 out of them
        pc_in_image_frame = pc2.create_cloud_xyz32(msg.header, points_on_plane_from_cam)

        # Lookup the transform from the camera to the field plane
        try:
            trans = self._tf_buffer.lookup_transform(
                self._publish_frame,
                self._camera_info.header.frame_id,
                msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(str(ex), throttle_duration_sec=5)
            return None

        # Transform the whole point cloud accordingly
        pc_relative = do_transform_cloud(pc_in_image_frame, trans)

        # Publish point cloud
        publisher.publish(pc_relative)

    def get_plane(self, stamp, object_height):
        """ returns a plane which an object is believed to be on as a tuple of a point on this plane and a normal"""

        base_frame = self._base_footprint_frame

        field_normal = PointStamped()
        field_normal.header.frame_id = base_frame
        field_normal.header.stamp = stamp
        field_normal.point.x = 0.0
        field_normal.point.y = 0.0
        field_normal.point.z = 1.0
        try:
            field_normal = self._tf_buffer.transform(field_normal,
                                                     self._camera_info.header.frame_id,
                                                     timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(str(ex), throttle_duration_sec=5)
            return None

        field_point = PointStamped()
        field_point.header.frame_id = base_frame
        field_point.header.stamp = stamp
        field_point.point.x = 0.0
        field_point.point.y = 0.0
        field_point.point.z = object_height
        try:
            field_point = self._tf_buffer.transform(field_point, self._camera_info.header.frame_id)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(str(ex), throttle_duration_sec=5)
            return None

        field_normal = np.array([field_normal.point.x, field_normal.point.y, field_normal.point.z])
        field_point = np.array([field_point.point.x, field_point.point.y, field_point.point.z])

        # field normal is a vector! so it stats at field point and goes up in z direction
        field_normal = field_point - field_normal
        return field_normal, field_point

    def _get_field_intersection_for_pixels(self, points, field, scale=1.0):
        """
        Projects an numpy array of points to the correspoding places on the field plane (in the camera frame).
        """
        camera_projection_matrix = self._camera_info.K

        binning_x = max(self._camera_info.binning_x, 1) / scale
        binning_y = max(self._camera_info.binning_y, 1) / scale

        points[:, 0] = (points[:, 0] - (camera_projection_matrix[2] / binning_x)) / (camera_projection_matrix[0] / binning_x)
        points[:, 1] = (points[:, 1] - (camera_projection_matrix[5] / binning_y)) / (camera_projection_matrix[4] / binning_y)
        points[:, 2] = 1

        intersections = self._line_plane_intersections(field[0], field[1], points)

        return intersections

    def _transform_point(self, point: Point, field, stamp) -> Point:
        np_point = self._get_field_intersection_for_pixels(np.array([[point.x, point.y, point.z]]), field)[0]

        if np.isnan(np_point).any():
            return None

        intersection_stamped = PointStamped()
        intersection_stamped.point.x = np_point[0]
        intersection_stamped.point.y = np_point[1]
        intersection_stamped.point.z = np_point[2]
        intersection_stamped.header.stamp = stamp
        intersection_stamped.header.frame_id = self._camera_info.header.frame_id

        try:
            intersection_transformed = self._tf_buffer.transform(intersection_stamped, self._publish_frame)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(str(ex), throttle_duration_sec=5)
            return None

        return intersection_transformed.point

    def _line_plane_intersections(self, plane_normal, plane_point, ray_directions):
        n_dot_u = np.tensordot(plane_normal, ray_directions, axes=([0],[1]))
        relative_ray_distance = -plane_normal.dot(- plane_point) / n_dot_u

        # we are casting a ray, intersections need to be in front of the camera
        relative_ray_distance[relative_ray_distance <= 0] = np.nan

        ray_directions[:,0] = np.multiply(relative_ray_distance, ray_directions[:,0])
        ray_directions[:,1] = np.multiply(relative_ray_distance, ray_directions[:,1])
        ray_directions[:,2] = np.multiply(relative_ray_distance, ray_directions[:,2])

        return ray_directions

    def _object_at_bottom_of_image(self, position, thresh):
        """
        Checks if the objects y position is at the bottom of the image.

        :param position: Y-position of the object
        :param thresh: Threshold defining the region at the bottom of the image which is counted as 'the bottom' as a fraction of the image height
        """
        image_height = self._camera_info.height / max(self._camera_info.binning_y, 1)
        scaled_thresh = thresh * image_height
        return position > scaled_thresh


def main(args=None):
    rclpy.init(args=args)
    node = Transformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()