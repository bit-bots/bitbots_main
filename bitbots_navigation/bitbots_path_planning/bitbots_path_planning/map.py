import shapely
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import Point
from rclpy.node import Node
from ros2_numpy import numpify
from shapely import Geometry
from tf2_geometry_msgs import PointStamped, PoseWithCovarianceStamped

CIRCLE_APPROXIMATION_SEGMENTS = 12


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
        self._obstacles: list[Geometry]
        self._ball_geometry = list[Geometry]
        self._obstacles_union: Geometry | None = None

    def obstacles(self) -> list[Geometry]:
        """
        Return the obstacles in the map as Geometry objects
        """
        return self._obstacles

    def intersects(self, object: Geometry) -> bool:
        """
        Check if an object intersects with any obstacles in the map
        """
        return not self._obstacles_union.touches(object) and self._obstacles_union.intersects(object)

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
        self._update_geometries()

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
        self._update_geometries()

    def _update_geometries(self):
        self._obstacles = []
        for robot in self.robot_buffer:
            center = shapely.Point(robot.bb.center.position.x, robot.bb.center.position.y)
            radius = max(numpify(robot.bb.size)[:2]) / 2
            dilation = self.config_inflation_dialation / self.node.config.map.resolution
            geometry = center.buffer(radius + dilation, quad_segs=CIRCLE_APPROXIMATION_SEGMENTS // 4)
            self._obstacles.append(geometry)
        if self.ball_obstacle_active:
            for ball in self.ball_buffer:
                center = shapely.Point(ball.x, ball.y)
                radius = self.config_ball_diameter / 2
                dilation = self.config_inflation_dialation / self.node.config.map.resolution
                geometry = center.buffer(radius + dilation, quad_segs=CIRCLE_APPROXIMATION_SEGMENTS // 4)
                self._obstacles.append(geometry)
        self._obstacles_union = shapely.union_all(self._obstacles)

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
