# -*- coding:utf-8 -*-
"""
FieldieSearchDecision
^^^^^^^^^^^^^^^^^^^^

This Module decides if the Fieldie should Search or Walk/Turn to Find the Ball

The Search is continued until someone interrupts it by reevaluate or interrupt

History:
''''''''

* 06.12.14 Created (Nils Rokita)

"""
import rospy

from bitbots_body_behaviour.body.actions.search import StopAndSearch
from bitbots_body_behaviour.body.actions.go_to import GoToCenterpoint, GoToRelativePosition

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class FieldieSearchDecision(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(FieldieSearchDecision, self).__init__(connector, _)
        self.start_time = rospy.get_time()
        self.turn_wait_time = connector.config["Body"]["Kieldie"]["searchWaitTime"]
        self.turns_before_going_to_center_point = connector.config["Body"]["Kieldie"]["turnCenterpointTime"]
        self.turn_angle = connector.config["Body"]["Fieldie"]["searchingTurnAngularAbsolute"]
        self.counter = 0

    def perform(self, connector, reevaluate=False):
        # if we have looked too long in the same direktion without seeing we want to walk
        if rospy.get_time() - self.start_time > self.turn_wait_time:
            # reset the timer and increment counter
            self.start_time = rospy.get_time()
            self.counter += 1
            return self.push(GoToRelativePosition, (0, 0, self.turn_angle))
        else:
            if self.counter >= self.turns_before_going_to_center_point:
                return self.push(GoToCenterpoint)
            else:
                # Just Search
                return self.push(StopAndSearch)
