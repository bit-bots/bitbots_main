#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class DummyImu(Node):

    def __init__(self):
        super().__init__('DummyImu')
        self.pub = self.create_publisher(Imu, '/imu/data', 1)
        self.msg = Imu()
        self.msg.header.frame_id = 'imu'
        self.msg.orientation.w = 1.0

    def loop(self):
        r = self.create_rate(100)
        while rclpy.ok():
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)
            r.sleep()


if __name__ == '__main__':
    rclpy.init(args=None)

    node = DummyImu()
    # necessary so that sleep in loop() is not blocking
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.loop()

    node.destroy_node()
    rclpy.shutdown()
