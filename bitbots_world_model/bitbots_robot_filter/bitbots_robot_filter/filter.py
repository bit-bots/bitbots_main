#! /usr/bin/env python3

import numpy as np
import rclpy
import soccer_vision_3d_msgs.msg as sv3dm
import soccer_vision_attribute_msgs.msg as svam
import tf2_geometry_msgs
import tf2_ros as tf2
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import numpify
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tf2_geometry_msgs import PoseStamped

from bitbots_msgs.msg import TeamData


class RobotFilter(Node):
    def __init__(self):
        super().__init__("bitbots_robot_filter")

        self.tf_buffer = Buffer(self, Duration(seconds=10.0))
        self.camera_info: CameraInfo | None = None
        self.logger = self.get_logger()

        self.robots: list[tuple[sv3dm.Robot, Time]] = list()
        self.team: dict[int, TeamData] = dict()

        self.base_footprint_frame = self.declare_parameter("base_footprint_frame", "base_footprint").value
        self.filter_frame = self.declare_parameter("filter_frame", "map").value
        self.robot_dummy_size = self.declare_parameter("robot_dummy_size", 0.4).value
        self.robot_merge_angle = self.declare_parameter("robot_merge_angle", 0.05).value  # TODO: Find a good value
        self.robot_storage_time = self.declare_parameter(
            "robot_storage_time", 10e9
        ).value  # TODO: increase value (maybe 60s)
        self.team_data_timeout = self.declare_parameter("team_data_timeout", 1e9).value

        self.create_subscription(
            sv3dm.RobotArray,
            self.declare_parameter("robot_observation_topic", "robots_relative").value,
            self._robot_vision_callback,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.create_subscription(
            TeamData,
            self.declare_parameter("team_data_topic", "team_data").value,
            self._team_data_callback,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "camera/camera_info", self.camera_info_callback, 1
        )  # TODO: add parameter (self.camera_info_topic)

        self.robot_obstacle_publisher = self.create_publisher(
            sv3dm.RobotArray, self.declare_parameter("robots_publish_topic", "robots_relative_filtered").value, 1
        )

        self.create_timer(1 / 20, self.publish_obstacles, callback_group=MutuallyExclusiveCallbackGroup())

    def publish_obstacles(self):
        # Set current timestamp and frame
        dummy_header = Header()
        dummy_header.stamp = self.get_clock().now().to_msg()
        dummy_header.frame_id = self.filter_frame

        # Cleanup robot obstacles
        self.robots = list(
            filter(
                lambda robot: abs((self.get_clock().now() - Time.from_msg(robot[1])).nanoseconds)
                < self.robot_storage_time,
                self.robots,
            )
        )

        # Convert TeamData to Robot Observation
        def build_robot_detection_from_team_data(msg: TeamData) -> sv3dm.Robot:
            robot = sv3dm.Robot()
            robot.bb.center = msg.robot_position.pose
            robot.bb.size.x = self.robot_dummy_size
            robot.bb.size.y = self.robot_dummy_size
            robot.bb.size.z = 1.0
            robot.attributes.team = svam.Robot.TEAM_OWN
            robot.attributes.player_number = msg.robot_id
            return robot

        def is_team_data_fresh(msg: TeamData) -> bool:
            return (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds < self.team_data_timeout

        # We don't need the time stamps and we want a new list, so the team data is only applied temporarily
        robots = [robot_msg for robot_msg, _ in self.robots]

        # Add Team Mates (if the data is fresh enough)
        robots.extend(map(build_robot_detection_from_team_data, filter(is_team_data_fresh, self.team.values())))

        # Publish the robot obstacles
        self.robot_obstacle_publisher.publish(sv3dm.RobotArray(header=dummy_header, robots=robots))

    def is_robot_in_fov(self, header: Header, robot: sv3dm.Robot) -> bool:
        """
        Calculates if the whole robot should be currently visible
        """
        # Check if we got a camera info to do this stuff
        if self.camera_info is None:
            self.logger.info("No camera info received. Not checking if the ball is currently visible.")
            return False

        # Build a robot pose
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self.filter_frame
        robot_pose.header.stamp = header.stamp
        robot_pose.pose.position = robot.bb.center.position

        # For the base footprint z is 0 and for the head z is the height
        for z in [0.0, robot.bb.size.z]:
            robot_pose.pose.position.z = z

            # Transform to camera frame
            try:
                robot_in_camera_optical_frame = self.tf_buffer.transform(
                    robot_pose, self.camera_info.header.frame_id, timeout=Duration(nanoseconds=0.5 * (10**9))
                )
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.logger.warning(str(e))
                return False

            # Check if the robot is in front of the camera
            if robot_in_camera_optical_frame.pose.position.z < 0:
                return False

            # Quick math to get the pixel
            p = numpify(robot_in_camera_optical_frame.pose.position)
            k = np.reshape(self.camera_info.k, (3, 3))
            pixel = np.matmul(k, p)
            pixel = pixel * (1 / pixel[2])

            # Make sure that the transformed pixel is on the sensor and not too close to the border
            border_fraction_horizontal = 0.1  # TODO: add parameter (self.negative_observation_ignore_border)
            border_px_horizontal = (
                np.array([self.camera_info.width, self.camera_info.height]) / 2 * border_fraction_horizontal
            )
            in_fov_horizontal = bool(
                border_px_horizontal[0] < pixel[0] <= self.camera_info.width - border_px_horizontal[0]
            )
            in_fov_vertical = bool(pixel[1] <= self.camera_info.height)
            if not (in_fov_horizontal and in_fov_vertical):
                return False

        # If we reach this point the robot is in the field of view
        return True

    def _robot_vision_callback(self, msg: sv3dm.RobotArray):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.filter_frame, msg.header.frame_id, msg.header.stamp, Duration(seconds=1.0)
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))
            return

        # Own position
        pos: Vector3 = self.tf_buffer.lookup_transform(
            self.filter_frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation

        old_robots = self.robots
        self.robots = list()
        robot: sv3dm.Robot
        for robot in msg.robots:
            # Transfrom robot to map frame
            robot.bb.center = tf2_geometry_msgs.do_transform_pose(robot.bb.center, transform)

            # Remove robots that are too close to the new robot or in the field of view
            old_robot: sv3dm.Robot
            for old_robot, stamp in old_robots.copy():
                angle_robot = np.arctan2(robot.bb.center.position.y - pos.y, robot.bb.center.position.x - pos.x)
                angle_old_robot = np.arctan2(
                    old_robot.bb.center.position.y - pos.y, old_robot.bb.center.position.x - pos.x
                )

                # Check if there is another robot in memory close to it regarding the angle or in the field of view
                if abs(angle_robot - angle_old_robot) < self.robot_merge_angle or self.is_robot_in_fov(
                    msg.header, old_robot
                ):
                    old_robots.remove((old_robot, stamp))

            # Append our new robots
            self.robots.append((robot, msg.header.stamp))
        # Extend our robot list with a list that does not contain duplicates and robots that are not in the field of view
        self.robots.extend(old_robots)

    def _team_data_callback(self, msg: TeamData):
        self.team[msg.robot_id] = msg

    def camera_info_callback(self, msg: CameraInfo):
        """Updates the camera intrinsics"""
        self.camera_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = RobotFilter()
    # Number of executor threads is the number of MutuallyExclusiveCallbackGroups + 2 threads the tf listener and executor needs
    ex = MultiThreadedExecutor(num_threads=5)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
