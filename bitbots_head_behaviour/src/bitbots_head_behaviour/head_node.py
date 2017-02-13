#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory

from humanoid_league_msgs.msg import HeadMode

from bitbots_head_behaviour.head_stack_machine import HeadStackMachine

from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider

from bitbots_stackmachine.stack_machine_module import StackMachineModule

from bitbots_head_behaviour.src.bitbots_head_behaviour.head_connector import HeadConnector


class HeadNode(StackMachineModule):

    def __init__(self):
        self.connector = HeadConnector()
        self.set_start_module(HeadDutyDecider)

        self.head_stack = HeadStackMachine()

        rospy.Subscriber("/head_duty", HeadMode, self.duty_cb, queue_size=10)


        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.head_stack.update()


    def duty_cb(self):
