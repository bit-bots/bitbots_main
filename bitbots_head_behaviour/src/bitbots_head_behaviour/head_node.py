#!/usr/bin/env python3
import rospy
from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider
from bitbots_head_behaviour.head_connector import HeadConnector
from bitbots_stackmachine.stack_machine_module import StackMachineModule
from humanoid_league_msgs.msg import HeadMode


class HeadNode(StackMachineModule):
    def __init__(self):
        super().__init__()
        self.connector = HeadConnector()
        self.set_start_module(HeadDutyDecider)

        rospy.Subscriber("/head_duty", HeadMode, self.duty_cb, queue_size=10)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.update()

    def duty_cb(self, mode: HeadMode):
        pass
