#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu


class DummyImu(Node):

    def __init__(self):
        super().__init__('DummyImu')
        self.pub = self.create_publisher(Imu, '/imu/data', 1)
        self.msg = Imu()
        self.msg.header.frame_id = 'imu'
        self.msg.orientation.w = 1.0

    def loop(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


if __name__ == '__main__':
    rclpy.init(args=None)

    node = DummyImu()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.create_timer(0.01, node.loop)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
