#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, EmptyRequest
from bitbots_buttons.msg import Buttons

rclpy.init(args=None)
zero_l = self.create_client(Empty, "/foot_pressure_left/set_foot_zero")
zero_r = self.create_client(Empty, "/foot_pressure_right/set_foot_zero")
button_prev_state = False
press_time = rospy.get_rostime() - Duration(seconds=1.0)

def cb(msg):
    global button_prev_state, press_time
    print("New msg")
    print(msg.button1)
    print(not button_prev_state)
    print(rospy.get_rostime() - press_time > Duration(seconds=1.0))
    if msg.button3 and not button_prev_state and rospy.get_rostime() - press_time > Duration(seconds=1.0):
        zero_l()
        zero_r()
        press_time = rospy.get_rostime()
    button_prev_state = msg.button3


rospy.Subscriber("/buttons", Buttons, cb, queue_size=1)
rclpy.spin(self)
