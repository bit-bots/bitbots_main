#!/usr/bin/env python3

from __future__ import print_function
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import DynUpGoal, DynUpAction, DynUpFeedback
import humanoid_league_msgs.msg
from sensor_msgs.msg import Imu

showing_feedback = False

if __name__ == "__main__":
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
        if len(sys.argv) > 1 and '--feedback' in sys.argv:
            print('Feedback')
            print(feedback)
            print()


    def play_animation(anim):
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
        goal.hcm = False
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


    def play_dynup(direction):
        global last_move_time
        goal = DynUpGoal()
        goal.direction = direction

        client.done_cb = done_cb
        client.feedback_cb = feedback_cb
        client.active_cb = active_cb
        client.send_goal_async(goal)
        print("Sent new goal. Waiting for result")
        client.wait_for_result()


    last_move_time = None


    def imu_cb(msg: Imu):
        global last_move_time
        if msg.angular_velocity.x > 0.15 or msg.angular_velocity.y > 0.15:
            last_move_time = self.get_clock().now().to_sec()


    rclpy.init(args=None)
    print('[..] Connecting to action server \'dynup\'', end='')
    sys.stdout.flush()
    client = ActionClient(self, DynUpAction, 'dynup')
    if not client.wait_for_server():
        exit(1)
    print('\r[OK] Connecting to action server \'dynup\'')

    anim_client = ActionClient(self, humanoid_league_msgs.msg.PlayAnimationAction, 'animation')

    imu_sub = self.create_subscription(Imu, "/imu/data", imu_cb, 1)

    while rclpy.ok():
        direction = None
        anim = None
        inp = input("Which direction [f|b]")
        if inp == "f":
            direction = "front"
            anim = "falling_front"
            key_anim = "stand_up_front"
        elif inp == "b":
            direction = "back"
            anim = "falling_back"
            key_anim = "stand_up_back"
        else:
            print(f"input {inp} not valid")
            continue

        input("Will now go into falling position. please hold robot and press enter")
        play_animation(anim)

        inp = input("Please put robot on the ground. Then choose dynup or keyframe [d|k]")
        if inp == "d":
            last_move_time = None
            start_time = self.get_clock().now().to_sec()
            play_dynup(direction)
        elif inp == "k":
            last_move_time = None
            start_time = self.get_clock().now().to_sec()
            play_animation(key_anim)
        else:
            print("invalid input")
            continue

        # wait till robot is standing at least 3 seconds not moving
        while last_move_time is None or self.get_clock().now().to_sec() - last_move_time < 3:
            self.get_clock().sleep_for(Duration(seconds=0.0001)
        # compute duration
        duration = last_move_time - start_time

        print(f"Dynup took {duration} s")
