#!/usr/bin/env python3
import cv2
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile


class FieldPublisher(Node):
    """
    A Publisher Node that loads and publishes the map of the playing field
    """

    def __init__(self) -> None:
        super().__init__("field_publisher")
        qos = QoSProfile(
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=5,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/field/map", qos)
        self.occupancy = None
        self.map = None
        self.resolution = None
        self.x = None
        self.y = None
        self.padding = None
        timer_period = 1
        self.load_map(
            get_package_share_directory("bitbots_parameter_blackboard") + "/config/fields/webots/lines.png",
            get_package_share_directory("bitbots_parameter_blackboard") + "/config/fields/webots/config.yaml",
        )
        self.timer = self.create_timer(timer_period, self.publish_map)

    def load_map(
        self,
        map_path: str,
        yaml_path: str,
    ) -> None:
        """
        loads the map and map config and saves it in this class

        args:
        map_path        path to the map .png file
        yaml_path       path to the config .yaml file
        """
        # read image and config
        self.map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        with open(yaml_path) as config_file:
            map_config = yaml.safe_load(config_file)

        self.x = map_config["parameter_blackboard"]["ros__parameters"]["field"]["size"]["x"]
        self.y = map_config["parameter_blackboard"]["ros__parameters"]["field"]["size"]["y"]
        self.padding = map_config["parameter_blackboard"]["ros__parameters"]["field"]["size"]["padding"]
        self.resolution = (
            map_config["parameter_blackboard"]["ros__parameters"]["field"]["size"]["y"]
            + 2 * map_config["parameter_blackboard"]["ros__parameters"]["field"]["size"]["padding"]
        ) / self.map.shape[0]

        # create new occupancy grid message
        self.occupancy = OccupancyGrid()
        self.occupancy.header.frame_id = "map"
        self.occupancy.info.resolution = self.resolution
        self.occupancy.info.width = self.map.shape[0]
        self.occupancy.info.height = self.map.shape[1]
        self.occupancy.info.origin.position.x = -1.5 + float(-self.map.shape[0] / 2 * self.occupancy.info.resolution)
        self.occupancy.info.origin.position.y = 1.5 + float(-self.map.shape[1] / 2 * self.occupancy.info.resolution)
        self.occupancy.info.origin.position.z = 0.0
        self.occupancy.info.origin.orientation.x = 0.7071068
        self.occupancy.info.origin.orientation.y = 0.7071068
        self.occupancy.info.origin.orientation.z = 0.0
        self.occupancy.info.origin.orientation.w = 0.0
        self.occupancy.data = self.map.flatten(order="F")  # .astype(np.int64)

    def publish_map(self) -> None:
        """-+
        publishes the most rescently loaded map
        """
        self.map_pub.publish(self.occupancy)


def main(args=None):
    rclpy.init(args=None)
    node = FieldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


main()
