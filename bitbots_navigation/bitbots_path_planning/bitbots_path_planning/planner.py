import typing
from abc import ABC, abstractmethod
from itertools import product

import geometry_msgs.msg as geom_msg
import rustworkx as rx
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import soccer_vision_3d_msgs.msg as sv3dm
from ros2_numpy import numpify

from bitbots_pathplanning_rust import ObstacleMapConfig, ObstacleMap, RoundObstacle


class Planner(ABC):
    @abstractmethod
    def set_goal(self, pose: PoseStamped) -> None:
        pass

    @abstractmethod
    def cancel_goal(self) -> None:
        pass

    @abstractmethod
    def set_robots(self, robots: sv3dm.RobotArray) -> None:
        pass

    @abstractmethod
    def set_ball(self, ball: PoseWithCovarianceStamped) -> None:
        pass

    @abstractmethod
    def avoid_ball(self, state: bool) -> None:
        pass

    @abstractmethod
    def active(self) -> bool:
        pass

    @abstractmethod
    def step(self) -> Path:
        pass


class VisibilityPlanner(Planner):
    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer
        self.robots = []
        self.ball = None
        self.goal: PoseStamped | None = None
        self.base_footprint_frame: str = self.node.config.base_footprint_frame
        self.ball_obstacle_active: bool = True
        self.frame: str = self.node.config.map.planning_frame

    def set_robots(self, robots: sv3dm.RobotArray):
        new_buffer: list[RoundObstacle] = []
        for robot in robots.robots:
            point = PointStamped()
            point.header.frame_id = robots.header.frame_id
            point.point = robot.bb.center.position
            radius = max(numpify(robot.bb.size)[:2]) / 2
            try:
                position = self.buffer.transform(point, self.frame).point
                new_buffer.append(RoundObstacle(center=(position.x, position.y), radius=radius))
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.node.get_logger().warn(str(e))
        self.robots = new_buffer

    def set_ball(self, ball: PoseWithCovarianceStamped) -> None:
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.position
        try:
            tf_point = self.buffer.transform(point, self.frame).point
            self.ball = RoundObstacle(center=(tf_point.x, tf_point.y), radius=self.node.config.map.ball_diameter)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.ball = None
            self.node.get_logger().warn(str(e))

    def set_goal(self, pose: PoseStamped) -> None:
        """
        Updates the goal pose
        """
        pose.header.stamp = Time(clock_type=self.node.get_clock().clock_type).to_msg()
        self.goal = pose

    def avoid_ball(self, state: bool) -> None:
        """
        Activates or deactivates the ball obstacle
        """
        self.ball_obstacle_active = state

    def cancel_goal(self) -> None:
        """
        Removes the current goal
        """
        self.goal = None
        self.path = None

    def active(self) -> bool:
        """
        Determine if we have an active goal
        """
        return self.goal is not None

    def step(self) -> Path:
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        my_position = self.buffer.lookup_transform(
            self.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation
        start = (my_position.x, my_position.y)
        config = ObstacleMapConfig(dilate=self.node.config.map.inflation.dilate, num_vertices=12)
        obstacles = self.robots
        if self.ball_obstacle_active and self.ball is not None:
            obstacles.append(self.ball)
        omap = ObstacleMap(config, obstacles)
        path = omap.shortest_path(start, goal)

        def map_to_pose(position):
            pose = PoseStamped()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            return pose

        return Path(
            header=Header(frame_id=self.frame, stamp=self.node.get_clock().now().to_msg()),
            poses=list(map(map_to_pose, path)),
        )

