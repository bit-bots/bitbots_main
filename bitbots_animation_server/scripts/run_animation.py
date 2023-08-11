#!/usr/bin/env python3
import sys

import rclpy
from actionlib_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from humanoid_league_msgs.action import PlayAnimation


def anim_run(anim=None, hcm=False):
    node = Node("run_animation")

    anim_client = ActionClient(node, PlayAnimation, 'animation')

    if anim is None or anim == "":
        node.get_logger().warn("Tried to play an animation with an empty name!")
        return False

    first_try = anim_client.wait_for_server(3.0)

    if not first_try:
        node.get_logger().error(
            "Animation Action Server not running! Motion can not work without animation action server. "
            "Will now wait until server is accessible!")
        anim_client.wait_for_server()
        node.get_logger().warn("Animation server now running, hcm will go on.")

    goal = PlayAnimation.Goal()
    goal.animation = anim
    goal.hcm = hcm

    # todo not .send_goal does never return
    state = anim_client.send_goal_async(goal)
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


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    # run with "ros2 run bitbots_animation_server run_animation.py NAME"
    if len(sys.argv) > 1:
        # Support for _anim:=NAME -style execution for legacy reasons
        if sys.argv[1].startswith('_anim:=') or sys.argv[1].startswith('anim:='):
            anim_run(sys.argv[1].split(':=')[1], 'hcm' in sys.argv)
        else:
            anim_run(sys.argv[1], 'hcm' in sys.argv)
