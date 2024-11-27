import numpy as np
import pyastar2d
import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from bitbots_path_planning.map import Map


class Planner:
    """
    A simple grid based A* interface
    """

    def __init__(self, node: Node, buffer: tf2.Buffer, map: Map) -> None:
        self.node = node
        self.buffer = buffer
        self.map = map
        self.goal: PoseStamped | None = None
        self.path: Path | None = None
        self.base_footprint_frame: str = self.node.config.base_footprint_frame

    def set_goal(self, pose: PoseStamped) -> None:
        """
        Updates the goal pose
        """
        pose.header.stamp = Time(clock_type=self.node.get_clock().clock_type).to_msg()
        self.goal = pose

    def cancel(self) -> None:
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
        Generates a new A* path to the goal pose with respect to the costmap
        """
        goal = self.goal

        # Get current costmap
        navigation_grid = self.map.get_map()

        # Get my pose and position on the map
        my_position = self.buffer.lookup_transform(
            self.map.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation

        # Transform goal pose to map frame if needed
        if goal.header.frame_id != self.map.frame:
            goal = self.buffer.transform(goal, self.map.frame, timeout=Duration(seconds=0.2))

        # Run A* from our current position to the goal position
        path = pyastar2d.astar_path(
            navigation_grid.astype(np.float32),
            self.map.to_map_space(my_position.x, my_position.y),
            self.map.to_map_space(goal.pose.position.x, goal.pose.position.y),
            allow_diagonal=False,
        )

        # Convert the pixel coordinates to world coordinates
        path = self.map.from_map_space_np(path)

        # Build path message
        def to_pose_msg(element):
            pose = PoseStamped()
            pose.pose.position.x = element[0]
            pose.pose.position.y = element[1]
            return pose

        poses = list(map(to_pose_msg, path))

        poses.append(goal)
        self.path = Path(
            header=Header(frame_id=self.map.get_frame(), stamp=self.node.get_clock().now().to_msg()), poses=poses
        )

        return self.path

    def get_path(self) -> Path | None:
        """
        Returns the most recent path
        """
        return self.path
