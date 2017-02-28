#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from humanoid_league_msgs.msg import Speak


def sender():
    pub = rospy.Publisher('speak', Speak, queue_size=10)
    rospy.init_node('send_text', anonymous=False)
    if rospy.has_param("~text") or rospy.has_param("~filename"):
        msg = Speak()
        if rospy.has_param("~text"):
            msg.text = rospy.get_param("~text")
        if rospy.has_param("~filename"):
            msg.filename = rospy.get_param("~filename")
        if rospy.has_param("~prio"):
            msg.priority = rospy.get_param("~prio")
        else:
            msg.priority = msg.MID_PRIORITY
        pub.publish(msg)
    else:
        print("Please specify Text by adding _text:=YOURTEXT at the end of rosrun")


if __name__ == '__main__':
    # run with "rosrun humanoid_league_speaker send_text.py _text:=TEXT"
    # you can add _prio:=NUMBER with a number between 0 and 2 to set the priority level
    # if you use _filename instead of _text you can give out a file with the corresponding path
    print("Sending Message")
    sender()
