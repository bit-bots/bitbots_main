#!/usr/bin/env python3
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from bitbots_msgs.msg import JointCommand, DynUpAction, DynUpGoal
import math


def callback(msg):
    output_string = "{\n \"author\": \"walkready_script\", \n \"description\": \"none\",\n \"hostname\": \"tams159\", \n \"keyframes\": [ \n { \n \"duration\": 1.0, \n \"goals\": { \n"
    i = 0
    for joint_name in msg.joint_names:
        output_string += " \"" + str(joint_name) + "\" : " + str(math.degrees(msg.positions[i]))
        if i < len(msg.joint_names) - 1:
            output_string += ","
        output_string += "\n"
        i += 1
    output_string += "},\n \"name\": \"generated frame\", \n \"pause\": 0.5 \n }\n ], \n \"name\": \"walkready\", \n\"version\": 0 \n} \n"
    with open("walkready.json", "w") as text_file:
        text_file.write(output_string)


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


rospy.init_node("walkready_script")
rospy.logwarn("Make sure that the Dynup is running together with simulator or this script will not work.")

rospy.Subscriber("/DynamixelController/command", JointCommand, callback)

print('[..] Connecting to action server \'dynup\'', end='')
sys.stdout.flush()
client = actionlib.SimpleActionClient('dynup', DynUpAction)
if not client.wait_for_server():
    exit(1)
print('\r[OK] Connecting to action server \'dynup\'')
print()

goal = DynUpGoal()
goal.direction = "rise"

client.send_goal(goal)
client.done_cb = done_cb
print("Sent new goal. Waiting for result")
client.wait_for_result()

rospy.loginfo("Your walkready animation has been saved to the current directory.")
