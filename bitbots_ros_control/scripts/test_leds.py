#!/usr/bin/env python3

import rclpy
from bitbots_msgs.srv import Leds
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

rclpy.init(args=None)

node = Node("test_leds")

client = node.create_client(Leds, "/set_leds")

if not client.wait_for_service(timeout_sec=10):
    node.get_logger().fatal("Service not available")
    exit()

request = Leds.Request()
for i in range(3):
    request.leds.append(ColorRGBA())
    request.leds[i].r = 1.0
    request.leds[i].g = 0.0
    request.leds[i].b = 0.0
    request.leds[i].a = 1.0


client.call(request)
