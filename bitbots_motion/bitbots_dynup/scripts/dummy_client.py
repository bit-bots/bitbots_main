#!/usr/bin/env python3

import sys

import rclpy
from actionlib_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from bitbots_msgs.action import Dynup

showing_feedback = False

if __name__ == "__main__":
    directions = [
        Dynup.Goal.DIRECTION_FRONT,
        Dynup.Goal.DIRECTION_FRONT_ONLY,
        Dynup.Goal.DIRECTION_BACK,
        Dynup.Goal.DIRECTION_BACK_ONLY,
        Dynup.Goal.DIRECTION_RISE,
        Dynup.Goal.DIRECTION_DESCEND,
        Dynup.Goal.DIRECTION_WALKREADY,
    ]
    if len(sys.argv) != 2 or sys.argv[1] not in directions:
        print("Use " + str(directions) + " as parameters!")
        sys.exit(1)

    print("[..] Initializing node", end="")
    rclpy.init(args=None)
    node = Node("dynup_dummy_client")
    print("\r[OK] Initializing node")

    def done_cb(state, result):
        print("Action completed: ", end="")
        if state == GoalStatus.PENDING:
            print("Pending")
        elif state == GoalStatus.ACTIVE:
            print("Active")
        elif state == GoalStatus.PREEMPTED:
            print("Preempted")
        elif state == GoalStatus.SUCCEEDED:
            print("Succeeded")
        elif state == GoalStatus.ABORTED:
            print("Aborted")
        elif state == GoalStatus.REJECTED:
            print("Rejected")
        elif state == GoalStatus.PREEMPTING:
            print("Preempting")
        elif state == GoalStatus.RECALLING:
            print("Recalling")
        elif state == GoalStatus.RECALLED:
            print("Recalled")
        elif state == GoalStatus.LOST:
            print("Lost")
        else:
            print("Unknown state", state)
        print(str(result))

    def active_cb():
        print("Server accepted action")

    def feedback_cb(feedback):
        if len(sys.argv) > 1 and "--feedback" in sys.argv:
            print("Feedback")
            print(feedback)
            print()

    print("[..] Connecting to action server 'dynup'", end="")
    sys.stdout.flush()
    client = ActionClient(node, Dynup, "dynup")
    if not client.wait_for_server():
        sys.exit(1)
    print("\r[OK] Connecting to action server 'dynup'")
    print()

    goal = Dynup.Goal()
    goal.direction = sys.argv[1]

    client.send_goal_async(goal)
    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    print("Sent new goal")
