#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from bitbots_speaker.msg import Speak


def sender():
    pub = rospy.Publisher('speak', Speak, queue_size=10)
    rospy.init_node('send_text', anonymous=False)
    if rospy.has_param("~text"):
        text = rospy.get_param("~text")
        msg = Speak()
        msg.text = text
        if rospy.has_param("~prio"):
            msg.priority = rospy.get_param("~prio")
        else:
            msg.priority = msg.MID_PRIORITY
        pub.publish(msg)
    else:
        print("Please specify Text by adding _text:=YOURTEXT at the end of rosrun")


if __name__ == '__main__':
    # run with "rosrun bitbots_speaker send_text.py _text:=TEXT"
    # you can add _prio:=NUMBER with a number between 0 and 2 to set the priority level
    print("Sending Message")
    sender()
