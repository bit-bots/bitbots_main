# -*- coding:utf-8 -*-
"""
OneTimeKickerDecision
^^^^^^^^^^^^^^^^^^^^^

Start of OneTimeKicker behaviour.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
import rospy

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.decisions.common.ball_seen import BallSeenFieldie


class OneTimeKickerDecision(AbstractDecisionModule):  # todo make this player shoot always with the hard kick
    def __init__(self, connector):
        super(OneTimeKickerDecision, self).__init__(connector)
        self.reset_time = connector.config["Behaviour"]["OneTimeKicker"]["resetTime"]

    def perform(self, connector, reevaluate=False):
        # sets the robot back to its original role if the time is up
        if connector.blackboard.get_one_time_kicker_timer() + self.reset_time < rospy.get_time():
            connector.set_duty(None)
            return self.interrupt()

        # sets the robot back to its original role if he has kicked
        if connector.blackboard.get_one_time_kicked():
            connector.set_duty(None)
            connector.blackboard.set_one_time_kicked(False)
            return self.interrupt()

        # if we stay in this duty, we will behave like a fieldie
        return self.push(BallSeenFieldie)

    def get_reevaluate(self):
        # is true to check the duty resetes
        return True
