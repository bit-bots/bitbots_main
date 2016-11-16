# -*- coding:utf-8 -*-
"""
HeadToPanTilt
^^^^^^^^^^^^^

.. moduleauthor:: Nils <0rokita@informatik.uni-hamburg.de>

This action moves the head to a given pan/tilt position and waits there for a second

This module expects a 2tupel containing pan and tilt for the head

History:

* 19.08.14: Created (Nils)

"""
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
import time
from bitbots.util.config import get_config


class HeadToPanTilt(AbstractActionModule):
    def __init__(self, args):
        super(HeadToPanTilt, self).__init__()
        self.pan = float(args[0])
        self.tilt = float(args[1])
        config = get_config()
        self.delta = config["Behaviour"]["Common"]["Search"]["headTurnPrecision"]
        self.wait_time = config["Behaviour"]["Common"]["Search"]["headTurnTime"]
        self.pan_speed = config["Behaviour"]["Common"]["Search"]["maxPanSpeedSearch"]
        self.tilt_speed = config["Behaviour"]["Common"]["Search"]["maxTiltSpeedSearch"]
        self.at_position = time.time()

    def perform(self, connector, reevaluate=False):
        ipc = connector.get_ipc()
        pose = ipc.get_pose()
        if abs(pose.head_pan.position - self.pan) < self.delta and abs(
                pose.head_tilt.position - self.tilt) < self.delta:
            # We reached the position
            if time.time() - self.at_position > self.wait_time:
                # We waited long enough, go back
                return self.pop()
        else:
            # We haven't reached it
            # Update when we should reach it
            self.at_position = time.time()
            pose.head_pan.goal = self.pan
            pose.head_pan.speed = self.pan_speed
            pose.head_tilt.goal = self.tilt
            pose.head_tilt.speed = self.tilt_speed
            ipc.update(pose)
