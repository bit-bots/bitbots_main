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
import time

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule

from bitbots_behaviour.src.bitbots_behaviour.model import Connector


class HeadToPanTilt(AbstractActionModule):
    def __init__(self, args):
        super(HeadToPanTilt, self).__init__()
        self.pan = float(args[0])
        self.tilt = float(args[1])
        self.at_position = time.time()

    def perform(self, connector: Connector, reevaluate=False):
        curren_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()

        if abs(curren_pan_pos - self.pan) < connector.head.delta and abs(current_tilt_pos - self.tilt) < self.connector.head.delta:
            # We reached the position
            if time.time() - self.at_position > connector.head.wait_time:
                # We waited long enough, go back
                return self.pop()
        else:
            # We haven't reached it
            # Update when we should reach it
            self.at_position = time.time()
            connector.head.send_motor_goals(self.pan, connector.head.pan_speed_max, self.tilt, connector.head.tilt_speed_max)
