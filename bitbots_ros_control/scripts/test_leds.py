#!/usr/bin/env python3

import rospy
from bitbots_msgs.srv import LedsRequest, Leds
from std_msgs.msg import ColorRGBA

rospy.init_node("leds_tester")
prox = rospy.ServiceProxy("/set_leds", Leds)

request = LedsRequest()
for i in range(3):
    request.leds.append(ColorRGBA())
    request.leds[i].r = 0.0
    request.leds[i].g = 0.0
    request.leds[i].b = 0.0
    request.leds[i].a = 1.0
    

prox(request)