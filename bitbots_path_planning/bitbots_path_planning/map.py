from typing import List, Tuple

import cv2
import numpy as np
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from ros2_numpy import msgify, numpify
from tf2_geometry_msgs import PointStamped

from humanoid_league_msgs.msg import PoseWithCertaintyStamped


class Map:
    """
    Costmap that keeps track of obstacles like the ball or other robots.
    """
    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer
        self.resolution: int = self.node.declare_parameter('map.resolution', 20).value
        self.size: Tuple[float, float] = (
            self.node.declare_parameter('map.size.x', 11.0).value,
            self.node.declare_parameter('map.size.y', 8.0).value
        )
        self.map: np.ndarray = np.ones(
            (np.array(self.size)*self.resolution).astype(int),
            dtype=np.int8)
        self.frame: str = self.node.declare_parameter('map.planning_frame', 'map').value
        self.ball_buffer: List[Point] = []
        self.robot_buffer: List[sv3dm.Robot] = []
        self.config_ball_diameter: float = self.node.declare_parameter('map.ball_diameter', 0.13).value
        self.config_inflation_blur: int = self.node.declare_parameter('map.inflation.blur', 13).value
        self.config_inflation_dialation: int = self.node.declare_parameter('map.inflation.dialte', 3).value
        self.config_obstacle_value: int = self.node.declare_parameter('map.obstacle_value', 50).value

    def set_ball(self, ball: PoseWithCertaintyStamped) -> None:
        """
        Adds a given ball to the ball buffer
        """
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.pose.position
        try:
            self.ball_buffer = [self.buffer.transform(point, self.frame).point]
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.node.get_logger().warn(str(e))

    def _render_balls(self) -> None:
        """
        Draws the ball buffer onto the costmap
        """
        ball: sv3dm.Ball
        for ball in self.ball_buffer:
            cv2.circle(
                self.map,
                self.to_map_space(ball.x, ball.y)[::-1],
                round(self.config_ball_diameter * self.resolution),
                self.config_obstacle_value,
                -1)

    def set_robots(self, robots: sv3dm.RobotArray):
        """
        Adds a given robot array to the robot buffer
        """
        new_buffer: List[sv3dm.Robot] = []
        robot: sv3dm.Robot
        for robot in robots.robots:
            point = PointStamped()
            point.header.frame_id = robots.header.frame_id
            point.point = robot.bb.center.position
            try:
                robot.bb.center.position = self.buffer.transform(point, self.frame).point
                new_buffer.append(robot)
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                    self.node.get_logger().warn(str(e))
        self.robot_buffer = new_buffer

    def _render_robots(self) -> None:
        """
        Draws the robot buffer onto the costmap
        """
        robot: sv3dm.Robot
        for robot in self.robot_buffer:
            cv2.circle(
                self.map,
                self.to_map_space(robot.bb.center.position.x, robot.bb.center.position.y)[::-1],
                round(max(numpify(robot.bb.size)[:2]) * self.resolution),
                self.config_obstacle_value,
                -1)

    def to_map_space(self, x: float, y: float) -> Tuple[int, int]:
        """
        Maps a point (x, y in meters) to corresponding pixel on the costmap
        """
        return (
            max(0, min(round((x - self.get_origin()[0]) * self.resolution), self.map.shape[0] - 1)),
            max(0, min(round((y - self.get_origin()[1]) * self.resolution), self.map.shape[1] - 1)),
        )

    def from_map_space_np(self, points: np.ndarray) -> np.ndarray:
        """
        Maps an array of pixel coordinates from the costmap to world coordinates (meters)
        """
        return points / self.resolution + self.get_origin()

    def get_origin(self) -> np.ndarray:
        """
        Origin of the costmap in meters
        """
        return np.array([
            -self.map.shape[0] / self.resolution / 2,
            -self.map.shape[1] / self.resolution / 2
        ])

    def clear(self) -> None:
        """
        Clears the complete cost map
        """
        self.map[...] = 1

    def inflate(self) -> None:
        """
        Applies inflation to all occupied areas of the costmap
        """
        idx = self.map == 1
        map = cv2.dilate(
            self.map.astype(np.uint8),
            np.ones((
                self.config_inflation_dialation,
                self.config_inflation_dialation), np.uint8),
            iterations=2)
        self.map[idx] = cv2.blur(
            map,
            (self.config_inflation_blur, self.config_inflation_blur)
        ).astype(np.int8)[idx]

    def update(self) -> None:
        """
        Regenerates the costmap based on the ball and robot buffer
        """
        self.clear()
        self._render_balls()
        self._render_robots()
        self.inflate()

    def get_map(self) -> np.ndarray:
        """
        Returns the costmap as an numpy array
        """
        return self.map

    def get_frame(self) -> str:
        """
        Returns the frame of reference of the map
        """
        return self.frame

    def to_msg(self) -> OccupancyGrid:
        """
        Returns the costmap as an OccupancyGrid message
        """
        msg: OccupancyGrid = msgify(OccupancyGrid, self.get_map().T)
        msg.header.frame_id = self.frame
        msg.info.width = self.map.shape[0]
        msg.info.height = self.map.shape[1]
        msg.info.resolution = 1 / self.resolution
        msg.info.origin.position.x = self.get_origin()[0]
        msg.info.origin.position.y = self.get_origin()[1]
        return msg
