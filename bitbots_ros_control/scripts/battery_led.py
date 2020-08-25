#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA, Float64

rospy.init_node("battery_led")

pub = rospy.Publisher("/led2", ColorRGBA, queue_size=1)

led_full = ColorRGBA()
led_full.a = 1.0
led_full.r = 0
led_full.g = 0
led_full.b = 1

led_mid = ColorRGBA()
led_mid.a = 1.0
led_mid.r = 0
led_mid.g = 1
led_mid.b = 0

led_low = ColorRGBA()
led_low.a = 1.0
led_low.r = 1
led_low.g = 0
led_low.b = 0

led_no = ColorRGBA()
led_no.a = 1.0
led_no.r = 0
led_no.g = 0
led_no.b = 0


def vbat_cb(msg: Float64):
    if msg.data > 16:
        pub.publish(led_full)
    elif msg.data > 14:
        pub.publish(led_mid)
    elif msg.data > 13:
        pub.publish(led_low)
    else:
        pub.publish(led_no)


sub = rospy.Subscriber("/core/vbat", Float64, vbat_cb, queue_size=1, tcp_nodelay=True)
rospy.spin()
