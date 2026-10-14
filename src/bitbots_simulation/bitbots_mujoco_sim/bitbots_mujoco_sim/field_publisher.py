import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from bitbots_utils.utils import get_parameters_from_other_node
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def field_map_msg(field_map: np.ndarray, frame_id: str) -> OccupancyGrid:
    """
    Convert a field map image into an occupancy grid centered on the frame origin.

    Mirrors the map message of the localization (see bitbots_localization Map::get_map_msg),
    so the field looks the same in RViz regardless of which one is running.
    """
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.resolution = 0.01  # The field maps use one pixel per centimeter
    msg.info.height, msg.info.width = field_map.shape
    msg.info.origin.position.x = -msg.info.width / 2.0 * msg.info.resolution
    msg.info.origin.position.y = -msg.info.height / 2.0 * msg.info.resolution
    msg.info.origin.orientation.w = 1.0
    msg.data = (100 - field_map.astype(np.int16)).astype(np.int8).flatten().tolist()
    return msg


class FieldPublisher(Node):
    """Publishes the line map of the current field, which is otherwise done by the localization."""

    def __init__(self) -> None:
        super().__init__("field_publisher")
        map_frame = self.declare_parameter("map_frame", "map").value

        field_name = get_parameters_from_other_node(self, "/parameter_blackboard", ["field.name"])["field.name"]
        map_path = os.path.join(
            get_package_share_directory("bitbots_parameter_blackboard"), "config", "fields", field_name, "lines.png"
        )
        field_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if field_map is None:
            raise FileNotFoundError(f"Could not load field map '{map_path}'")

        publisher = self.create_publisher(
            OccupancyGrid, "field/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        publisher.publish(field_map_msg(field_map, map_frame))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
