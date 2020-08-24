#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyRequest
from bitbots_buttons.msg import Buttons

rospy.init_node("zero_feet_on_buttons")
zero_l = rospy.ServiceProxy("/foot_pressure_left/set_foot_zero", Empty)
zero_r = rospy.ServiceProxy("/foot_pressure_right/set_foot_zero", Empty)
button_prev_state = False
press_time = rospy.get_rostime() - rospy.Duration(1.0)

def cb(msg):
    global button_prev_state, press_time
    print("New msg")
    print(msg.button1)
    print(not button_prev_state)
    print(rospy.get_rostime() - press_time > rospy.Duration(1.0))
    if msg.button3 and not button_prev_state and rospy.get_rostime() - press_time > rospy.Duration(1.0):
        zero_l()
        zero_r()
        press_time = rospy.get_rostime()
    button_prev_state = msg.button3


rospy.Subscriber("/buttons", Buttons, cb, queue_size=1)
rospy.spin()
