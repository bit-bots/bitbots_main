import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class BitbotsMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("map_publisher")
        self._map_publisher = self.create_publisher(OccupancyGrid, "field/map", 1)


def main(args=None):
    rclpy.init(args=args)

    bitbots_map_publisher = BitbotsMapPublisher()

    rclpy.spin(bitbots_map_publisher)

    bitbots_map_publisher.destroy_node()
    rclpy.shutdown()
