#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float64

rclpy.init(args=None)
node = Node("battery_led")

pub = node.create_publisher(ColorRGBA, "/led2", 1)

led_full = ColorRGBA()
led_full.a = 1.0
led_full.r = 0.0
led_full.g = 0.0
led_full.b = 1.0

led_mid = ColorRGBA()
led_mid.a = 1.0
led_mid.r = 0.0
led_mid.g = 1.0
led_mid.b = 0.0

led_low = ColorRGBA()
led_low.a = 1.0
led_low.r = 1.0
led_low.g = 0.0
led_low.b = 0.0

led_no = ColorRGBA()
led_no.a = 1.0
led_no.r = 0.0
led_no.g = 0.0
led_no.b = 0.0


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
