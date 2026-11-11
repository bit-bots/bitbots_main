import copy
from abc import ABC, abstractmethod
from typing import Any, Optional

import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_global_planner import (
    NavigationBounds,
    ObstacleMap,
    ObstacleMapConfig,
    PolygonObstacle,
    RoundObstacle,
)
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.time import Time
from ros2_numpy import numpify
from std_msgs.msg import ColorRGBA, Header
from tf2_geometry_msgs import PointStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

from bitbots_path_planning import NodeWithConfig


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
    def __init__(self, node: NodeWithConfig, buffer: tf2.BufferInterface) -> None:
        self.node = node
        self.buffer = buffer
        self.robots: list[RoundObstacle] = []
        self.ball: Optional[RoundObstacle] = None
        self.goal: Optional[PoseStamped] = None
        self.base_footprint_frame: str = self.node.config.base_footprint_frame
        self.ball_obstacle_active: bool = True
        self.frame: str = self.node.config.map.planning_frame

        # Get the field dimensions from the global parameter blackboard
        field_parameters = get_parameters_from_other_node(
            self.node,
            "/parameter_blackboard",
            [
                "field.size.x",
                "field.size.y",
                "field.size.padding",
                "field.goal.width",
                "field.goal.depth",
                "field.navigation_bounds.x_min",
                "field.navigation_bounds.x_max",
                "field.navigation_bounds.y_min",
                "field.navigation_bounds.y_max",
            ],
        )
        # Create static obstacles for both goals, so we don't plan paths through them
        self.goal_obstacles: list[PolygonObstacle] = self._create_goal_obstacles(
            field_length=field_parameters["field.size.x"],
            goal_width=field_parameters["field.goal.width"],
            goal_depth=field_parameters["field.goal.depth"],
        )
        # Hard limits for the robot position that planned paths never leave
        self.navigation_bounds: Optional[NavigationBounds] = self._create_navigation_bounds(
            {name: field_parameters[f"field.navigation_bounds.{name}"] for name in ("x_min", "x_max", "y_min", "y_max")}
        )
        # Half of the field size including the padding, used to visualize unbounded directions
        padding = field_parameters["field.size.padding"]
        self.field_half_length_with_padding: float = field_parameters["field.size.x"] / 2 + padding
        self.field_half_width_with_padding: float = field_parameters["field.size.y"] / 2 + padding

    def navigation_bounds_marker(self) -> Optional[Marker]:
        """
        Creates a marker showing each configured navigation bound as a line.
        The lines end at the other bounds, or at the field border including the padding,
        if there is no bound in that direction.
        Returns None if no navigation bounds are configured.
        """
        if self.navigation_bounds is None:
            return None
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.ns = "navigation_bounds"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        bounds = self.navigation_bounds
        x_low = bounds.x_min if bounds.x_min is not None else -self.field_half_length_with_padding
        x_high = bounds.x_max if bounds.x_max is not None else self.field_half_length_with_padding
        y_low = bounds.y_min if bounds.y_min is not None else -self.field_half_width_with_padding
        y_high = bounds.y_max if bounds.y_max is not None else self.field_half_width_with_padding
        for x in (bounds.x_min, bounds.x_max):
            if x is not None:
                marker.points.extend([Point(x=x, y=y_low), Point(x=x, y=y_high)])
        for y in (bounds.y_min, bounds.y_max):
            if y is not None:
                marker.points.extend([Point(x=x_low, y=y), Point(x=x_high, y=y)])
        return marker

    def _create_navigation_bounds(self, limits: dict[str, Any]) -> Optional[NavigationBounds]:
        """
        Creates the navigation bounds from the field config limits (x_min, x_max, y_min, y_max).
        Each limit is optional, if none is set, the planning is not bounded.
        Invalid values raise an exception, as planning with a misconfigured boundary is unsafe.
        """
        for name, value in limits.items():
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
                raise ValueError(f"Parameter field.navigation_bounds.{name} must be a number, got {value!r}")
        if all(value is None for value in limits.values()):
            self.node.get_logger().info("No navigation bounds configured for this field, planning is not bounded")
            return None
        bounds = NavigationBounds(**{name: None if value is None else float(value) for name, value in limits.items()})
        self.node.get_logger().info(f"Planning with hard navigation bounds from the field config: {bounds}")
        return bounds

    def _create_goal_obstacles(
        self, field_length: float, goal_width: float, goal_depth: float
    ) -> list[PolygonObstacle]:
        """
        Creates a U-shaped obstacle for each goal covering its sides and back.
        The back wall is extended away from the field, so we don't plan paths through
        the narrow space behind the goal.
        """
        wall_width = self.node.config.map.goal_obstacle.width
        back_extension = self.node.config.map.goal_obstacle.back_extension

        x_goal_line = field_length / 2
        x_back_inner = x_goal_line + goal_depth
        x_back_outer = x_back_inner + wall_width + back_extension
        y_inner = goal_width / 2
        y_outer = y_inner + wall_width

        # U-shaped polygon opening towards the field (here for the goal in positive x direction)
        vertices = [
            (x_goal_line, -y_outer),
            (x_back_outer, -y_outer),
            (x_back_outer, y_outer),
            (x_goal_line, y_outer),
            (x_goal_line, y_inner),
            (x_back_inner, y_inner),
            (x_back_inner, -y_inner),
            (x_goal_line, -y_inner),
        ]
        # Place one goal obstacle on each side of the field (mirrored in x)
        return [PolygonObstacle([(side * x, y) for x, y in vertices]) for side in (-1, 1)]

    def set_robots(self, robots: sv3dm.RobotArray):
        new_buffer: list[RoundObstacle] = []
        for robot in robots.robots:
            point = PointStamped()
            point.header.frame_id = robots.header.frame_id
            point.point = robot.bb.center.position
            # Use the maximum dimension of the bounding box as the radius
            radius = max(numpify(robot.bb.size)[:2]) / 2
            try:
                # Transform the point to the planning frame
                position = self.buffer.transform(point, self.frame).point
                # Add the robot to the buffer if the transformation was successful
                new_buffer.append(RoundObstacle(center=(position.x, position.y), radius=radius))
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.node.get_logger().warn(str(e))
        self.robots = new_buffer

    def set_ball(self, ball: PoseWithCovarianceStamped) -> None:
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.position
        try:
            # Transform the point to the planning frame
            tf_point = self.buffer.transform(point, self.frame).point
            # Create a new ball obstacle
            self.ball = RoundObstacle(center=(tf_point.x, tf_point.y), radius=self.node.config.map.ball_diameter / 2.0)
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
        """
        Computes the next path to the goal
        """
        assert self.goal is not None, "No goal set"
        # Define goal
        goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        # Get our current position
        my_position = self.buffer.lookup_transform(
            self.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation
        start = (my_position.x, my_position.y)

        # Configure how obstacles are represented
        config = ObstacleMapConfig(
            robot_radius=self.node.config.map.inflation.robot_radius,
            margin=self.node.config.map.inflation.obstacle_margin,
            num_vertices=12,
        )
        # Add robots to obstacles
        obstacles: list[RoundObstacle | PolygonObstacle] = list(self.robots)
        # Add ball to obstacles if active
        if self.ball is not None:
            obstacles.append(self.ball)
        # Add the static goal obstacles
        obstacles.extend(self.goal_obstacles)
        obstacle_map = ObstacleMap(config, obstacles, self.navigation_bounds)

        # Report when the navigation bounds change what we are doing
        if self.navigation_bounds is not None:
            if not self.navigation_bounds.contains(goal):
                self.node.get_logger().warn(
                    f"Goal {goal} is outside of the navigation bounds {self.navigation_bounds}, "
                    f"going to the closest allowed position {self.navigation_bounds.clamp(goal)} instead",
                    throttle_duration_sec=2.0,
                )
            if not self.navigation_bounds.contains(start):
                self.node.get_logger().warn(
                    f"Robot position {start} is outside of the navigation bounds {self.navigation_bounds}, "
                    "planning a path back into them",
                    throttle_duration_sec=2.0,
                )

        # Calculate the shortest path (its last point is the goal clamped into the navigation bounds)
        path = obstacle_map.shortest_path(start, goal)

        # Convert the path to a ROS messages
        def map_to_pose(position):
            pose = PoseStamped()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            return pose

        # The final pose keeps the requested goal orientation, but uses the planned
        # end position, which differs from the goal if it was outside of the navigation bounds
        final_pose = copy.deepcopy(self.goal)
        final_pose.pose.position.x, final_pose.pose.position.y = path[-1]

        # Generate the path message
        return Path(
            header=Header(frame_id=self.frame, stamp=self.node.get_clock().now().to_msg()),
            poses=list(map(map_to_pose, path)) + [final_pose],
        )
