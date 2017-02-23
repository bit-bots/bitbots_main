# -*- coding:utf-8 -*-
"""
OneTimeKickerDecision
^^^^^^^^^^^^^^^^^^^^^

Start of OneTimeKicker behaviour.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
import time

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class OneTimeKickerDecision(AbstractDecisionModule):  # todo make this player shoot always with the hard kick

    def __init__(self, _):
        super(OneTimeKickerDecision, self).__init__()
        config = get_config()
        self.reset_time = config["Behaviour"]["OneTimeKicker"]["resetTime"]

    def perform(self, connector, reevaluate=False):
        # sets the robot back to its original role if the time is up
        if connector.blackboard_capsule().get_one_time_kicker_timer() + self.reset_time < time.time():
            connector.set_duty(None)
            return self.interrupt()

        # sets the robot back to its original role if he has kicked
        if connector.blackboard_capsule().get_one_time_kicked():
            connector.set_duty(None)
            connector.blackboard_capsule().set_one_time_kicked(False)
            return self.interrupt()

        # if we stay in this duty, we will behave like a fieldie
        return self.push(BallSeenFieldie)

    def get_reevaluate(self):
        # is true to check the duty resetes
        return True
