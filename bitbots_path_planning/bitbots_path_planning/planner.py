from typing import Optional

import numpy as np
import pyastar2d
import tf2_geometry_msgs  # Is there for tf to recognize geometry_msgs even if it is not used explicitly
import tf2_ros as tf2
from bitbots_path_planning.map import Map
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header


class Planner:
    """
    A simple grid based A* interface
    """
    def __init__(self, node: Node, buffer: tf2.Buffer, map: Map) -> None:
        self.node = node
        self.buffer = buffer
        self.map = map
        self.goal: Optional[PoseStamped] = None
        self.path: Optional[Path] = None
        self.base_footprint_frame: str = self.node.get_parameter("base_footprint_frame").value

    def set_goal(self, pose: PoseStamped) -> None:
        """
        Updates the goal pose
        """
        pose.header.stamp = Time().to_msg()
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
        my_pose = self.buffer.lookup_transform(
            self.map.frame,
            self.base_footprint_frame,
            Time()).transform
        my_position = my_pose.translation

        # Transform goal pose to map frame if needed
        if goal.header.frame_id != self.map.frame:
            goal = self.buffer.transform(goal, self.map.frame)

        # Run A* from our current position to the goal position
        path = pyastar2d.astar_path(
            navigation_grid.astype(np.float32),
            self.map.to_map_space(
                my_position.x, my_position.y),
            self.map.to_map_space(
                goal.pose.position.x,
                goal.pose.position.y),
            allow_diagonal=False)

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
            header=Header(
                frame_id=self.map.get_frame(),
                stamp=self.node.get_clock().now().to_msg()),
            poses=poses)

        return self.path

    def get_path(self) -> Optional[Path]:
        """
        Returns the most recent path
        """
        return self.path
