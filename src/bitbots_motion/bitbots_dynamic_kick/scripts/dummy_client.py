#!/usr/bin/env python3

import argparse
import math
import os
import sys

import rclpy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from bitbots_msgs.action import Kick

showing_feedback = False

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("ball_y", type=float, help="y position of the ball [m]", default=0)
    parser.add_argument("kick_direction", type=float, help="kick direction [°]", default=0)
    parser.add_argument("-u", "--unstable", action="store_true", help="Use unstable kick")
    args = parser.parse_args()

    print(
        "Beware: this script may only work when calling it directly on the robot "
        "and will maybe result in tf errors otherwise"
    )
    print("[..] Initializing node", end="")
    rclpy.init(args=None)
    node = Node("dummy_client")
    marker_pub = node.create_publisher(Marker, "debug/dynamic_kick_ball_marker", 1)
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
        if len(sys.argv) > 1 and sys.argv[1] == "--feedback":
            print("Feedback")
            print(feedback)
            print()

    print("[..] Connecting to action server 'dynamic_kick'", end="")
    sys.stdout.flush()
    client = ActionClient(node, Kick, "dynamic_kick")
    if not client.wait_for_server():
        sys.exit(1)
    print("\r[OK] Connecting to action server 'dynamic_kick'")
    print()

    goal = Kick.Goal()
    goal.header.stamp = node.get_clock().now().to_msg()
    frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
    goal.header.frame_id = frame_prefix + "base_footprint"
    goal.ball_position.x = 0.2
    goal.ball_position.y = args.ball_y
    goal.ball_position.z = 0.0
    goal.unstable = args.unstable

    quat_vector = quaternion_from_euler(0, 0, math.radians(args.kick_direction))
    goal.kick_direction = Quaternion(x=quat_vector[0], y=quat_vector[1], z=quat_vector[2], w=quat_vector[3])

    goal.kick_speed = 6.7 if args.unstable else 1.0

    """marker = Marker()
    marker.header.stamp = goal.ball_position.header.stamp
    marker.header.frame_id = goal.ball_position.header.frame_id
    marker.pose.position = goal.ball_position.vector
    marker.pose.orientation.w = 1
    marker.scale = Vector3(0.05, 0.05, 0.05)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.lifetime = Duration(seconds=8)
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.id = 1
    marker.frame_locked = True
    marker_pub.publish(marker)
    """
    future = client.send_goal_async(goal)
    future.add_done_callback(done_cb)
    # future.add_feedback_callback(feedback_cb)
    print("Sent new goal. Waiting for result")
    # rclpy.spin_until_future_complete(client, future)
