from abc import ABC, abstractmethod

import numpy as np
import pyastar2d
import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped
from heapdict import heapdict
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shapely import LineString, Point, distance
from std_msgs.msg import Header

from bitbots_path_planning.map import Map


class Planner(ABC):
    @abstractmethod
    def set_goal(self, pose: PoseStamped) -> None:
        pass

    @abstractmethod
    def cancel_goal(self) -> None:
        pass

    @abstractmethod
    def active(self) -> bool:
        pass

    @abstractmethod
    def step(self) -> Path:
        pass


class VisibilityNode:
    """
    A Node that stores all the information needed for A-Star
    """

    def __init__(self, point: Point, cost=float("inf"), predecessor=None):
        """
        Initializes a Node at given Point
        """
        self.point = point
        self.cost = cost
        self.predecessor = predecessor


class VisibilityPlanner(Planner):
    def __init__(self, node: Node, buffer: tf2.Buffer, map: Map) -> None:
        self.node = node
        self.buffer = buffer
        self.map = map
        self.goal: PoseStamped | None = None
        self.base_footprint_frame: str = self.node.config.base_footprint_frame

    def set_goal(self, pose: PoseStamped) -> None:
        """
        Updates the goal pose
        """
        pose.header.stamp = Time(clock_type=self.node.get_clock().clock_type).to_msg()
        self.goal = pose

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
        goal = Point(self.goal.pose.position.x, self.goal.pose.position.y)
        my_position = self.buffer.lookup_transform(
            self.map.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation
        start = Point(my_position.x, my_position.y)

        open_list = heapdict()
        possible_successor_nodes = [VisibilityNode(start, 0.0), VisibilityNode(goal)]

        for obstacle in self.map.obstacles():
            for x, y in set(obstacle.boundary.coords):
                possible_successor_nodes.append(VisibilityNode(Point(x, y)))

        open_list[possible_successor_nodes[0]] = distance(start, goal)

        while len(open_list) != 0:
            current_node = open_list.popitem()
            if current_node[0].point == goal:
                path = [goal]
                next_node = current_node[0].predecessor
                while not next_node.point == start:
                    path.append(next_node.point)
                    next_node = next_node.predecessor
                path.append(start)
                poses = []
                for pos in reversed(path):
                    pose = PoseStamped()
                    pose.pose.position.x = pos.x
                    pose.pose.position.y = pos.y
                    poses.append(pose)
                poses.append(self.goal)
                return Path(
                    header=Header(frame_id=self.map.get_frame(), stamp=self.node.get_clock().now().to_msg()),
                    poses=poses,
                )  # create and return final LineString
            possible_successor_nodes.remove(current_node[0])

            # expand current node
            successors = []
            for node in possible_successor_nodes:  # find all correct successors of current node
                if not (self.map.intersects(LineString([current_node[0].point, node.point]))):
                    successors.append(node)
            for successor in successors:
                tentative_g = current_node[0].cost + distance(current_node[0].point, successor.point)
                if successor in open_list and tentative_g >= successor.cost:
                    continue
                successor.predecessor = current_node[0]
                successor.cost = tentative_g
                f = tentative_g + distance(node.point, goal)  # calculate new f-value (costs + heuristic)
                open_list[successor] = f


class GridPlanner(Planner):
    """
    A simple grid based A* interface
    """

    def __init__(self, node: Node, buffer: tf2.Buffer, map: Map) -> None:
        self.node = node
        self.buffer = buffer
        self.map = map
        self.goal: PoseStamped | None = None
        self.base_footprint_frame: str = self.node.config.base_footprint_frame

    def set_goal(self, pose: PoseStamped) -> None:
        """
        Updates the goal pose
        """
        pose.header.stamp = Time(clock_type=self.node.get_clock().clock_type).to_msg()
        self.goal = pose

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
        Generates a new A* path to the goal pose with respect to the costmap
        """
        goal = self.goal

        # Get current costmap
        navigation_grid = self.map.costmap

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
        return Path(
            header=Header(frame_id=self.map.get_frame(), stamp=self.node.get_clock().now().to_msg()), poses=poses
        )
