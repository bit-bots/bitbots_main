import math

import rclpy
import yaml  # type: ignore[import-untyped]
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Quaternion
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Semi-transparent orange, so obstacles read as markers rather than solid scenery
# and the planned path stays visible where it grazes them.
_OBSTACLE_COLOR = ColorRGBA(r=0.9, g=0.5, b=0.1, a=0.6)


class ObstacleMapPublisher(Node):
    """
    Publishes a static set of box/cylinder obstacles from a yaml file as a
    ``visualization_msgs/MarkerArray``.

    The same MarkerArray is consumed by :class:`VisibilityPlanner` (CUBE markers
    become rectangular obstacles, CYLINDER markers become round ones) and
    rendered directly in RViz, so one file both defines the obstacles the path
    planner avoids and visualizes them.

    The yaml file looks like::

        # optional; the ``frame`` node parameter is used when this is omitted
        frame: map
        obstacles:
          - {type: box, x: 1.0, y: 0.0, yaw: 0.3, size_x: 0.5, size_y: 0.8, height: 1.0}
          - {type: cylinder, x: 2.0, y: 1.0, radius: 0.3, height: 1.0}

    ``yaw`` (radians) and ``height`` are optional and default to 0.0 and 1.0.
    Positions are the obstacle centers on the ground, in ``frame``.
    """

    def __init__(self) -> None:
        super().__init__("obstacle_map")

        self._frame = self.declare_parameter("frame", "map").value
        topic = self.declare_parameter("obstacle_map_topic", "obstacle_map").value
        path = self.declare_parameter("obstacle_map_path", "").value
        # Republished at this rate so a planner that starts late still gets the
        # map (and RViz keeps showing it). 0 disables the timer (publish once).
        publish_rate = self.declare_parameter("publish_rate", 1.0).value

        # Latched, so a subscriber that joins between republish ticks still gets
        # the current map immediately.
        self._pub = self.create_publisher(
            MarkerArray, topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self._markers = self._load(path)
        self.get_logger().info(
            f"Publishing {len(self._markers.markers)} obstacle(s) in frame '{self._frame}' on '{topic}'."
        )
        self._publish()
        if publish_rate > 0.0:
            self.create_timer(1.0 / publish_rate, self._publish)

    def _load(self, path: str) -> MarkerArray:
        if not path:
            self.get_logger().warn("No 'obstacle_map_path' given; publishing an empty obstacle map.")
            return MarkerArray()

        with open(path) as f:
            data = yaml.safe_load(f) or {}

        # A frame set in the file wins over the node default, so a map file can
        # be fully self-contained.
        self._frame = data.get("frame", self._frame)

        markers = MarkerArray()
        for index, obstacle in enumerate(data.get("obstacles", [])):
            markers.markers.append(self._make_marker(index, obstacle))
        return markers

    def _make_marker(self, index: int, obstacle: dict) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame
        marker.ns = "obstacles"
        marker.id = index
        marker.action = Marker.ADD
        marker.color = _OBSTACLE_COLOR
        # 0 means "never auto-expire"; the map is static and republished anyway.
        marker.lifetime = DurationMsg(sec=0, nanosec=0)
        marker.frame_locked = True

        obstacle_type = obstacle["type"]
        height = float(obstacle.get("height", 1.0))
        marker.pose.position.x = float(obstacle["x"])
        marker.pose.position.y = float(obstacle["y"])
        # A CUBE/CYLINDER marker is centered on its pose, so lift it by half its
        # height to make it stand on the ground plane.
        marker.pose.position.z = height / 2.0
        marker.pose.orientation = _yaw_to_quaternion(float(obstacle.get("yaw", 0.0)))

        if obstacle_type == "box":
            marker.type = Marker.CUBE
            marker.scale.x = float(obstacle["size_x"])
            marker.scale.y = float(obstacle["size_y"])
            marker.scale.z = height
        elif obstacle_type == "cylinder":
            marker.type = Marker.CYLINDER
            diameter = 2.0 * float(obstacle["radius"])
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = height
        else:
            raise ValueError(f"Unknown obstacle type '{obstacle_type}' at index {index}; expected 'box' or 'cylinder'.")
        return marker

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self._markers.markers:
            marker.header.stamp = stamp
        self._pub.publish(self._markers)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapPublisher()

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
