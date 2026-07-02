from abc import ABC, abstractmethod
from typing import Optional

import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_rust_nav import ObstacleMap, ObstacleMapConfig, PolygonObstacle, RoundObstacle
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.time import Time
from ros2_numpy import numpify
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped, PoseWithCovarianceStamped

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
            self.node, "/parameter_blackboard", ["field.size.x", "field.goal.width", "field.goal.depth"]
        )
        # Create static obstacles for both goals, so we don't plan paths through them
        self.goal_obstacles: list[PolygonObstacle] = self._create_goal_obstacles(
            field_length=field_parameters["field.size.x"],
            goal_width=field_parameters["field.goal.width"],
            goal_depth=field_parameters["field.goal.depth"],
        )

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
        obstacle_map = ObstacleMap(config, obstacles)

        # Calculate the shortest path
        path = obstacle_map.shortest_path(start, goal)

        # Convert the path to a ROS messages
        def map_to_pose(position):
            pose = PoseStamped()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            return pose

        # Generate the path message
        return Path(
            header=Header(frame_id=self.frame, stamp=self.node.get_clock().now().to_msg()),
            poses=list(map(map_to_pose, path)) + [self.goal],
        )
