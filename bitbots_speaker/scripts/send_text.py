#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy


def sender(text):
    pub = rospy.Publisher('speak', Speak, queue_size=10)
    rospy.init_node('send_text', anonymous=False)
    rospy.Message()


if __name__ == '__main__':
