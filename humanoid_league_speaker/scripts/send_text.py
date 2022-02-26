#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import argparse

import rclpy
from humanoid_league_msgs.msg import Audio

if __name__ == '__main__':
    rclpy.init(args=None)
    node = rclpy.create_node('send_text')
    pub = node.create_publisher(Audio, 'speak', 10)
    args = argparse.ArgumentParser()
    args.add_argument("text", type=str)
    args.add_argument("--priority", type=int , default=None)
    args = args.parse_args()

    msg = Audio()
    msg.text = args.text
    if args.priority:
        msg.priority = args.priority
    else:
        msg.priority = 1
    pub.publish(msg)
