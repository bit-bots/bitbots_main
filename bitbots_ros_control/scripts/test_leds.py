#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bitbots_msgs.srv import LedsRequest, Leds
from std_msgs.msg import ColorRGBA

rclpy.init(args=None)
prox = self.create_client(Leds, "/set_leds")

request = LedsRequest()
for i in range(3):
    request.leds.append(ColorRGBA())
    request.leds[i].r = 1.0
    request.leds[i].g = 0.0
    request.leds[i].b = 0.0
    request.leds[i].a = 1.0
    

prox(request)