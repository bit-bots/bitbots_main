#!/usr/bin/env python3

import actionlib
import argparse
import math
import os
import random
import rospy
import sys
from time import sleep

from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.srv import SetObjectPose, SetObjectPosition, SetObjectPoseRequest, SetObjectPositionRequest
from geometry_msgs.msg import Vector3, Quaternion
from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler
import sys, select, termios, tty

showing_feedback = False

msg = """
BitBots Interactive Kick Test                              
-----------------------------
Move Robot:            Move Ball:
    q    w    e              t    
    a    s    d         f    g    h

SHIFT increases with factor 10

y/Y: set kick y command
x/X: set kick x command
c/C: set direction command
v/V: set speed command 

<: execute kick
r: reset robot and ball








"""
moveRobotBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    'W': (10, 0, 0),
    'S': (-10, 0, 0),
    'A': (0, 10, 0),
    'D': (0, -10, 0),
    'Q': (0, 0, 10),
    'E': (0, 0, -10),
}

moveBallBindings = {
    't': (1, 0, 0),
    'g': (-1, 0, 0),
    'f': (0, 1, 0),
    'h': (0, -1, 0),
    'T': (10, 0, 0),
    'G': (-10, 0, 0),
    'F': (0, 10, 0),
    'H': (0, -10, 0),
}

settings = termios.tcgetattr(sys.stdin)


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    rospy.init_node('dynamic_kick_interactive_test', anonymous=True)
    print("Waiting for kick server and simulation")

    def done_cb(state, result):
        return
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
        return
        print("Server accepted action")


    def feedback_cb(feedback):
        return
        if len(sys.argv) > 1 and sys.argv[1] == '--feedback':
            print('Feedback')
            print(feedback)
            print()


    sys.stdout.flush()
    client = actionlib.SimpleActionClient('dynamic_kick', KickAction)
    if not client.wait_for_server():
        exit(1)

    robot_x = 2.8
    robot_y = 0
    robot_yaw = 0
    ball_x = 3
    ball_y = 0
    kick_x = 0.1
    kick_y = 0
    kick_direction = 0
    kick_speed = 1

    x_speed_step = 0.01
    y_speed_step = 0.01
    turn_speed_step = 0.01

    frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"


    def generate_kick_goal(x, y, direction, speed, unstable=False):
        kick_goal = KickGoal()
        kick_goal.header.stamp = rospy.Time.now()
        kick_goal.header.frame_id = frame_prefix + "base_footprint"
        kick_goal.ball_position.x = x
        kick_goal.ball_position.y = y
        kick_goal.ball_position.z = 0
        kick_goal.kick_direction = Quaternion(*quaternion_from_euler(0, 0, direction))
        kick_goal.kick_speed = speed
        kick_goal.unstable = unstable
        return kick_goal


    def execute_kick():
        goal = generate_kick_goal(kick_x, kick_y, kick_direction, kick_speed)
        client.send_goal(goal)
        client.done_cb = done_cb
        client.feedback_cb = feedback_cb
        client.active_cb = active_cb
        client.wait_for_result()


    rospy.wait_for_service("set_robot_pose")
    set_robot_pose_service = rospy.ServiceProxy("set_robot_pose", SetObjectPose)
    rospy.wait_for_service("set_ball_position")
    set_ball_pos_service = rospy.ServiceProxy("set_ball_position", SetObjectPosition)


    def set_robot_pose():
        request = SetObjectPoseRequest()
        request.object_name = "amy"
        request.pose.position.x = robot_x
        request.pose.position.y = robot_y
        request.pose.position.z = 0.40
        request.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, robot_yaw))
        response = set_robot_pose_service(request)


    def set_ball_position():
        request = SetObjectPositionRequest()
        request.position.x = ball_x
        request.position.y = ball_y
        request.position.z = 0.04
        response = set_ball_pos_service(request)

    sys.stdout.write("\x1b[A")
    sys.stdout.write("\x1b[A")
    print(msg)

    while not rospy.is_shutdown():
        key = getKey()
        if key in moveRobotBindings.keys():
            robot_x += moveRobotBindings[key][0] * x_speed_step
            robot_x = round(robot_x, 4)
            robot_y += moveRobotBindings[key][1] * y_speed_step
            robot_y = round(robot_y, 4)
            robot_yaw += moveRobotBindings[key][2] * turn_speed_step
            robot_yaw = round(robot_yaw, 4)
            set_robot_pose()
        elif key in moveBallBindings.keys():
            ball_x += moveBallBindings[key][0] * x_speed_step
            ball_x = round(ball_x, 4)
            ball_y += moveBallBindings[key][1] * y_speed_step
            ball_y = round(ball_y, 4)
            set_ball_position()
        elif key == "<":
            execute_kick()
        elif key == "y":
            kick_y -= 0.01
        elif key == "Y":
            kick_y += 0.01
        elif key == "x":
            kick_x -= 0.01
        elif key == "X":
            kick_x += 0.01
        elif key == "c":
            kick_direction -= 0.01
        elif key == "C":
            kick_direction += 0.01
        elif key == "v":
            kick_speed -= 0.1
        elif key == "V":
            kick_speed += 0.1
        elif key == "r":
            set_robot_pose()
            set_ball_position()
        elif (key == '\x03'):
            break


        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        sys.stdout.write("\x1b[A")
        print(
            f"robot_x:    {round(robot_x, 2)}         ball_x:     {round(ball_x, 2)} \n"
            f"robot_y:    {round(robot_y, 2)}         ball_x:     {round(ball_y, 2)} \n"
            f"robot_yaw:  {round(robot_yaw, 2)}          \n"
            f"kick x:     {round(kick_x, 2)}             \n"
            f"kick y:     {round(kick_y, 2)}             \n"
            f"kick dir:   {round(kick_direction, 2)}     \n"
            f"kick speed: {round(kick_speed, 2)}         ")
