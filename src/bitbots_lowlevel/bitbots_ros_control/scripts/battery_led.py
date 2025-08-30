#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float64

rclpy.init(args=None)
node = Node("battery_led")

pub = node.create_publisher(ColorRGBA, "/led2", 1)

led_full = ColorRGBA(a=1.0, r=0.0, g=0.0, b=1.0)
led_mid = ColorRGBA(a=1.0, r=0.0, g=1.0, b=0.0)
led_low = ColorRGBA(a=1.0, r=1.0, g=0.0, b=0.0)
led_no = ColorRGBA(a=1.0, r=0.0, g=0.0, b=0.0)


def vbat_cb(msg: Float64):
    if msg.data > 16:
        pub.publish(led_full)
    elif msg.data > 14:
        pub.publish(led_mid)
    elif msg.data > 13:
        pub.publish(led_low)
    else:
        pub.publish(led_no)


sub = node.create_subscription(Float64, "/core/vbat", vbat_cb, 1)
rclpy.spin(node)
