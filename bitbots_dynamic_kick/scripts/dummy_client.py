#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib

from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback

rospy.init_node('dynamic_kick_dummy_client')

client = actionlib.SimpleActionClient('dynamic_kick', KickAction)

print('Waiting for server...')
if not client.wait_for_server():
    exit(1)
print('done')

goal = KickGoal()
goal.foot_speed = 0.5

def done_cb(state, result):
    print('Done!')
    print(state)
    print(result)

def active_cb():
    print('Active!')

def feedback_cb(feedback):
    print('Feedback')
    print(feedback)
    print()

client.send_goal(goal, done_cb, active_cb, feedback_cb)

client.wait_for_result()
