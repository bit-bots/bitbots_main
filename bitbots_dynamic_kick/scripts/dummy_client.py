#!/usr/bin/env python

from __future__ import print_function
import sys

from time import sleep
import random
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Vector3, Quaternion
from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler

showing_feedback = False

if __name__ == "__main__":
    print("Beware: this script may only work when calling it directly on the robot "
          "and will maybe result in tf errors otherwise")
    print("[..] Initializing node", end='')
    rospy.init_node('dynamic_kick_dummy_client', anonymous=True)
    marker_pub = rospy.Publisher("/debug/dynamic_kick_ball_marker", Marker, queue_size=1)
    print("\r[OK] Initializing node")


    def done_cb(state, result):
        print('Action completed: ', end='')
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)
        print(str(result))


    def active_cb():
        print("Server accepted action")


    def feedback_cb(feedback):
        if len(sys.argv) > 1 and sys.argv[1] == '--feedback':
            print('Feedback')
            print(feedback)
            print()


    print('[..] Connecting to action server \'dynamic_kick\'', end='')
    sys.stdout.flush()
    client = actionlib.SimpleActionClient('dynamic_kick', KickAction)
    if not client.wait_for_server():
        exit(1)
    print('\r[OK] Connecting to action server \'dynamic_kick\'')
    print()

    goal = KickGoal()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_footprint"
    goal.ball_position.x = 0.2
    goal.ball_position.y = -0.09
    goal.ball_position.z = 0

    goal.kick_direction = Quaternion(*quaternion_from_euler(0, 0, 0))

    goal.kick_speed = 1

    """marker = Marker()
    marker.header.stamp = goal.ball_position.header.stamp
    marker.header.frame_id = goal.ball_position.header.frame_id
    marker.pose.position = goal.ball_position.vector
    marker.pose.orientation.w = 1
    marker.scale = Vector3(0.05, 0.05, 0.05)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(8)
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.id = 1
    marker.frame_locked = True
    marker_pub.publish(marker)
"""
    client.send_goal(goal)
    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    print("Sent new goal. Waiting for result")
    client.wait_for_result()
