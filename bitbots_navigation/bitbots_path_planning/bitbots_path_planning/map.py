import cv2
import numpy as np
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from ros2_numpy import msgify, numpify
from tf2_geometry_msgs import PointStamped, PoseWithCovarianceStamped


class Map:
    """
    Costmap that keeps track of obstacles like the ball or other robots.
    """

    def __init__(self, node: Node, buffer: tf2.Buffer) -> None:
        self.node = node
        self.buffer = buffer
        parameters = get_parameters_from_other_node(
            self.node, "/parameter_blackboard", ["field.size.x", "field.size.y", "field.size.padding"]
        )
        self.size: tuple[float, float] = (
            parameters["field.size.x"] + 2 * parameters["field.size.padding"],
            parameters["field.size.y"] + 2 * parameters["field.size.padding"],
        )

        self.frame: str = self.node.config.map.planning_frame
        self.ball_buffer: list[Point] = []
        self.robot_buffer: list[sv3dm.Robot] = []
        self.config_ball_diameter: float = self.node.config.map.ball_diameter
        self.config_inflation_blur: int = self.node.config.map.inflation.blur
        self.config_inflation_dialation: int = self.node.config.map.inflation.dialate
        self.config_obstacle_value: int = self.node.config.map.obstacle_value
        self.ball_obstacle_active: bool = True

    def set_ball(self, ball: PoseWithCovarianceStamped) -> None:
        """
        Adds a given ball to the ball buffer
        """
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.position
        try:
            self.ball_buffer = [self.buffer.transform(point, self.frame).point]
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.node.get_logger().warn(str(e))

    def _render_balls(self, map: np.ndarray) -> None:
        """
        Draws the ball buffer onto the costmap
        """
        ball: sv3dm.Ball
        for ball in self.ball_buffer:
            cv2.circle(
                map,
                self.to_map_space(ball.x, ball.y)[::-1],
                round(self.config_ball_diameter / 2 * self.node.config.map.resolution),
                self.config_obstacle_value,
                -1,
            )

    def set_robots(self, robots: sv3dm.RobotArray):
        """
        Adds a given robot array to the robot buffer
        """
        new_buffer: list[sv3dm.Robot] = []
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

    def _render_robots(self, map: np.ndarray) -> None:
        """
        Draws the robot buffer onto the costmap
        """
        robot: sv3dm.Robot
        for robot in self.robot_buffer:
            cv2.circle(
                map,
                self.to_map_space(robot.bb.center.position.x, robot.bb.center.position.y)[::-1],
                round(max(numpify(robot.bb.size)[:2]) * self.node.config.map.resolution),
                self.config_obstacle_value,
                -1,
            )

    def to_map_space(self, x: float, y: float) -> tuple[int, int]:
        """
        Maps a point (x, y in meters) to corresponding pixel on the costmap
        """
        return (
            max(
                0,
                min(
                    round((x - self.get_origin()[0]) * self.node.config.map.resolution),
                    self.size[0] * self.node.config.map.resolution - 1,
                ),
            ),
            max(
                0,
                min(
                    round((y - self.get_origin()[1]) * self.node.config.map.resolution),
                    self.size[1] * self.node.config.map.resolution - 1,
                ),
            ),
        )

    def from_map_space_np(self, points: np.ndarray) -> np.ndarray:
        """
        Maps an array of pixel coordinates from the costmap to world coordinates (meters)
        """
        return points / self.node.config.map.resolution + self.get_origin()

    def get_origin(self) -> np.ndarray:
        """
        Origin of the costmap in meters
        """
        return np.array(
            [
                -(self.size[0] * self.node.config.map.resolution) / self.node.config.map.resolution / 2,
                -(self.size[1] * self.node.config.map.resolution) / self.node.config.map.resolution / 2,
            ]
        )

    @property
    def costmap(self) -> np.ndarray:
        """
        Generate the complete cost map
        """
        map: np.ndarray = np.ones((np.array(self.size) * self.node.config.map.resolution).astype(int), dtype=np.int8)
        if self.ball_obstacle_active:
            self._render_balls(map)
        self._render_robots(map)
        return self.inflate(map)

    def inflate(self, map: np.ndarray) -> np.ndarray:
        """
        Applies inflation to all occupied areas of the costmap
        """
        idx = map == 1
        nmap = cv2.dilate(
            map.astype(np.uint8),
            np.ones((self.config_inflation_dialation, self.config_inflation_dialation), np.uint8),
            iterations=2,
        )
        map[idx] = cv2.blur(nmap, (self.config_inflation_blur, self.config_inflation_blur)).astype(np.int8)[idx]
        return map

    def avoid_ball(self, state: bool) -> None:
        """
        Activates or deactivates the ball obstacle
        """
        self.ball_obstacle_active = state

    def get_frame(self) -> str:
        """
        Returns the frame of reference of the map
        """
        return self.frame

    def to_msg(self) -> OccupancyGrid:
        """
        Returns the costmap as an OccupancyGrid message
        """
        map = self.costmap
        msg: OccupancyGrid = msgify(OccupancyGrid, map)
        msg.header.frame_id = self.frame
        msg.info.width = map.shape[0]
        msg.info.height = map.shape[1]
        msg.info.resolution = 1 / self.node.config.map.resolution
        msg.info.origin.position.x = self.get_origin()[0]
        msg.info.origin.position.y = self.get_origin()[1]
        return msg
