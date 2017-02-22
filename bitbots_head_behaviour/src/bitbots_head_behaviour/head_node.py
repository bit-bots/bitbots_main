#!/usr/bin/env python3
import rospy
from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider
from bitbots_misc.bitbots_common.src.bitbots_common.connector.connector import Connector
from bitbots_stackmachine.stack_machine_module import StackMachineModule
from humanoid_league_msgs.msg import HeadMode


class HeadNode(StackMachineModule):
    def __init__(self):
        super().__init__()
        self.connector = Connector()
        self.set_start_module(HeadDutyDecider)

        rospy.Subscriber("/head_duty", HeadMode, self.connector.head.cb_headmode, queue_size=10)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.update()


