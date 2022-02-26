#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

if __name__ == "__main__":
    rclpy.init(args=None)
    pub = self.create_publisher(Imu, "test", 1)

    msg = Imu()
    rate = self.create_rate(100)

    while rclpy.ok():
        msg.header.stamp = self.get_clock().now()
        pub.publish(msg)
        rate.sleep()
