#!/usr/bin/env python3
from requests import request
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from bitbots_msgs.msg import Buttons
from rclpy.node import Node
from rclpy.duration import Duration

rclpy.init(args=None)
node = Node('zero_on_button')

zero_l = node.create_client(Empty, "/foot_pressure_left/set_foot_zero")
zero_r = node.create_client(Empty, "/foot_pressure_right/set_foot_zero")
button_prev_state = False
press_time = node.get_clock().now() - Duration(seconds=1.0)

def cb(msg):
    global button_prev_state, press_time
    print("New msg")
    print(msg.button1)
    print(not button_prev_state)
    print(node.get_clock().now() - press_time > Duration(seconds=1.0))
    if msg.button3 and not button_prev_state and node.get_clock().now() - press_time > Duration(seconds=1.0):
        request = Empty.Request()
        zero_l.call_async(request)
        zero_r.call_async(request)
        press_time = node.get_clock().now()
    button_prev_state = msg.button3


node.create_subscription(Buttons, "/buttons", cb, 1)
rclpy.spin(node)
