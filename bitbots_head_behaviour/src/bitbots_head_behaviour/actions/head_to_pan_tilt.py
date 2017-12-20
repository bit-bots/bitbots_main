"""
HeadToPanTilt
^^^^^^^^^^^^^

.. moduleauthor:: Nils <0rokita@informatik.uni-hamburg.de>

This action moves the head to a given pan/tilt position and waits there for the time given in head_config.yaml

This module expects a 2tupel containing pan and tilt for the head

"""
import time

import rospy

from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class HeadToPanTilt(AbstractActionModule):
    def __init__(self, connector: HeadConnector, args):
        super(HeadToPanTilt, self).__init__(connector)
        self.pan = float(args[0])
        self.tilt = float(args[1])
        self.at_position = time.time()

    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("HeadToPanTilt")
        current_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()

        if abs(current_pan_pos - self.pan) < connector.head.delta and \
                        abs(current_tilt_pos - self.tilt) < connector.head.delta:
            # We reached the position
            if time.time() - self.at_position > connector.head.wait_time:
                # We waited long enough, go back
                return self.pop()
        else:
            # We haven't reached it
            # Update when we should reach it
            pan_fitting = min(max(connector.head.min_pan, self.pan), connector.head.max_pan)
            tilt_fitting = min(max(connector.head.min_tilt, self.tilt), connector.head.max_tilt)
            self.at_position = time.time()
            rospy.logdebug("pan: " + str(pan_fitting) + " tilt:" + str(tilt_fitting))
            connector.head.send_motor_goals(pan_fitting, connector.head.pan_speed_max, tilt_fitting, connector.head.tilt_speed_max)
