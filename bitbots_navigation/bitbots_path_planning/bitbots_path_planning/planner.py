import typing
from abc import ABC, abstractmethod
from itertools import product

import geometry_msgs.msg as geom_msg
import rustworkx as rx
import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shapely import LineString, Point, distance
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

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

    def visibility_graph(self, start: Point, goal: Point) -> typing.Tuple[rx.PyGraph, int, int]:
        graph = rx.PyGraph()
        start_node = graph.add_node(start)
        goal_node = graph.add_node(goal)
        for obstacle in self.map.obstacles():
            for x, y in obstacle.boundary.coords:
                graph.add_node(Point(x, y))
        for a, b in product(graph.node_indices(), repeat=2):
            if a == b:
                continue
            a_point = graph.get_node_data(a)
            b_point = graph.get_node_data(b)
            if not self.map.intersects(LineString([a_point, b_point])):
                graph.add_edge(a, b, distance(a_point, b_point))
        return (graph, start_node, goal_node)

    def visibility_graph_wrapper(self) -> MarkerArray:
        goal = Point(self.goal.pose.position.x, self.goal.pose.position.y)
        my_position = self.buffer.lookup_transform(
            self.map.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation
        start = Point(my_position.x, my_position.y)
        (graph, _, _) = self.visibility_graph(start, goal)
        markers = MarkerArray()
        nodes = Marker(type=Marker.POINTS, ns="visibility_graph_nodes", action=Marker.ADD)
        nodes.header.frame_id = self.map.get_frame()
        nodes.header.stamp = self.node.get_clock().now().to_msg()
        for node in graph.nodes():
            nodes.points.append(geom_msg.Point(x=node.x, y=node.y, z=0.5))
        nodes.color.a = 1.0
        nodes.color.r = 0.2
        nodes.color.g = 0.2
        nodes.color.b = 0.2
        nodes.scale.x = 0.03
        nodes.scale.y = 0.03
        nodes.scale.z = 0.03
        markers.markers.append(nodes)
        edges = Marker(type=Marker.LINE_LIST, ns="visibility_graph_edges", action=Marker.ADD)
        edges.header.frame_id = self.map.get_frame()
        edges.header.stamp = self.node.get_clock().now().to_msg()
        for edge in graph.edge_indices():
            (a, b) = graph.get_edge_endpoints_by_index(edge)
            a_point = graph.get_node_data(a)
            b_point = graph.get_node_data(b)
            edges.points.append(geom_msg.Point(x=a_point.x, y=a_point.y, z=0.5))
            edges.points.append(geom_msg.Point(x=b_point.x, y=b_point.y, z=0.5))
        edges.color.a = 1.0
        edges.color.r = 0.2
        edges.color.g = 0.2
        edges.color.b = 0.2
        edges.scale.x = 0.015
        markers.markers.append(edges)
        return markers

    def step(self) -> Path:
        goal = Point(self.goal.pose.position.x, self.goal.pose.position.y)
        my_position = self.buffer.lookup_transform(
            self.map.frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
        ).transform.translation
        start = Point(my_position.x, my_position.y)
        (graph, start_node, goal_node) = self.visibility_graph(start, goal)
        path = rx.astar_shortest_path(
            graph, start_node, lambda node: node == goal, lambda edge: edge, lambda node: distance(node, goal)
        )

        def map_to_pose(node: int):
            position = graph.get_node_data(node)
            pose = PoseStamped()
            pose.pose.position.x = position.x
            pose.pose.position.y = position.y
            return pose

        return Path(
            header=Header(frame_id=self.map.get_frame(), stamp=self.node.get_clock().now().to_msg()),
            poses=list(map(map_to_pose, path)),
        )


"""
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
"""
