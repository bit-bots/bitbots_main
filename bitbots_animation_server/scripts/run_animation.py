#!/usr/bin/env python3
from rclpy.action import ActionClient
from actionlib_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
import sys

import humanoid_league_msgs.msg


def anim_run(anim=None, hcm=False):
    anim_client = ActionClient(self, humanoid_league_msgs.msg.PlayAnimationAction, 'animation')
    rclpy.init(args=None)
    if anim is None or anim == "":
        self.get_logger().warn("Tried to play an animation with an empty name!")
        return False
    first_try = anim_client.wait_for_server(
        Duration(seconds=self.get_parameter('"hcm/anim_server_wait_time"').get_parameter_value().double_value
    if not first_try:
        self.get_logger().error(
            "Animation Action Server not running! Motion can not work without animation action server. "
            "Will now wait until server is accessible!")
        anim_client.wait_for_server()
        self.get_logger().warn("Animation server now running, hcm will go on.")
    goal = humanoid_league_msgs.msg.PlayAnimationGoal()
    goal.animation = anim
    goal.hcm = hcm
    state = anim_client.send_goal_and_wait(goal)
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
    # run with "rosrun bitbots_animation_server run_animation.py NAME"
    if len(sys.argv) > 1:
        # Support for _anim:=NAME -style execution for legacy reasons
        if sys.argv[1].startswith('_anim:=') or sys.argv[1].startswith('anim:='):
            anim_run(sys.argv[1].split(':=')[1], 'hcm' in sys.argv)
        else:
            anim_run(sys.argv[1], 'hcm' in sys.argv)
