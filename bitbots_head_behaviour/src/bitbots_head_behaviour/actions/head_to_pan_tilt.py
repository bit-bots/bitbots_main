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
        # The head should not try to move to a position it cannot reach
        self.pan = min(max(connector.head.min_pan, float(args[0])), connector.head.max_pan)
        self.tilt = min(max(connector.head.min_tilt, float(args[1])), connector.head.max_tilt)
        # TODO: move body when ball is too far left or right
        self.at_position = rospy.get_time()

    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("HeadToPanTilt")
        current_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()

        if abs(current_pan_pos - self.pan) < connector.head.delta and \
                        abs(current_tilt_pos - self.tilt) < connector.head.delta:
            # We reached the position
            if rospy.get_time() - self.at_position > connector.head.wait_time:
                # We waited long enough, go back
                return self.pop()
        else:
            # We haven't reached it
            # Update when we should reach it
            self.at_position = rospy.get_time()
            rospy.logdebug("pan: " + str(self.pan) + " tilt:" + str(self.tilt))
            connector.head.send_motor_goals(self.pan, connector.head.pan_speed_max, self.tilt, connector.head.tilt_speed_max)
