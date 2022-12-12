from typing import Optional

import numpy as np
import pyastar2d
import tf2_geometry_msgs   # Is there for tf to recognize geomety_msgs even if it is not used explicitly
import tf2_ros as tf2
from bitbots_path_planning.map import Map
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header


class Planner:
    def __init__(self, node: Node, buffer: tf2.Buffer, map: Map) -> None:
        self.node = node
        self.buffer = buffer
        self.map = map
        self.goal = None
        self.path = None

    def set_goal(self, pose: PoseStamped) -> None:
        self.goal = pose

    def cancel(self) -> None:
        self.goal = None

    def active(self) -> bool:
        return self.goal is not None

    def step(self) -> Path:
        navigation_grid = self.map.get_map()

        my_pose = self.buffer.lookup_transform(self.map.frame, "base_footprint", Time()).transform
        my_position = my_pose.translation

        goal_pose_stamped = self.buffer.transform(self.goal, self.map.frame)

        path = pyastar2d.astar_path(
            navigation_grid.astype(np.float32),
            self.map.to_map_space(
                my_position.x, my_position.y),
            self.map.to_map_space(
                goal_pose_stamped.pose.position.x,
                goal_pose_stamped.pose.position.y),
            allow_diagonal=False)
        path = self.map.from_map_space_np(path)

        def to_pose_msg(element):
            pose = PoseStamped()
            pose.pose.position.x = element[0]
            pose.pose.position.y = element[1]
            return pose

        poses = list(map(to_pose_msg, path))

        poses.append(self.goal)
        self.path = Path(
            header=Header(
                frame_id=self.map.get_frame(),
                stamp=self.node.get_clock().now().to_msg()),
            poses=poses)

        return self.path

    def get_path(self) -> Optional[Path]:
        return self.path
