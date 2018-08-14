# -*- coding:utf-8 -*-
"""
OneTimeKickerDecision
^^^^^^^^^^^^^^^^^^^^^

Start of OneTimeKicker behaviour.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
import rospy

from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_body_behaviour.body.decisions.common.ball_seen import BallSeenFieldie


class OneTimeKickerDecision(AbstractDecisionElement):  # todo make this player shoot always with the hard kick
    def __init__(self, connector):
        super(OneTimeKickerDecision, self).__init__(connector)
        self.reset_time = connector.config["Body"]["OneTimeKicker"]["resetTime"]

    def perform(self, connector, reevaluate=False):
        # sets the robot back to its original role if the time is up
        if connector.blackboard.get_one_time_kicker_timer() + self.reset_time < rospy.get_time():
            connector.blackboard.set_is_one_time_kicker(True)
            return self.interrupt()

        # sets the robot back to its original role if he has kicked
        if connector.blackboard.get_one_time_kicked():
            connector.blackboard.set_one_time_kicked(False)
            connector.blackboard.set_is_one_time_kicker(False)
            return self.interrupt()

        # if we stay in this duty, we will behave like a fieldie
        return self.push(BallSeenFieldie)

    def get_reevaluate(self):
        # is true to check the duty resetes
        return True
