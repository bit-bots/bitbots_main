"""
FieldieSearchDecision
^^^^^^^^^^^^^^^^^^^^

This Module decides if the Fieldie Sould Search or Walk/Turn to Find the Ball

The Search is continued untel someone interrupts it by reevaluate or interrupt

History:
''''''''

* 06.12.14 Created (Nils Rokita)

"""
import time

from bitbots_body_behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots_body_behaviour.body.actions.search import StopAndSearch

from bitbots_common.connector.capsules.walking_capsule import WalkingCapsule
from bitbots_common.connector.connector import BodyConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class FieldieSearchDecision(AbstractDecisionModule):
    def __init__(self, connector: BodyConnector, _):
        super(FieldieSearchDecision, self).__init__(connector, _)
        self.start_time = time.time()
        self.turn_wait_time = connector.config["Head"]["Search"]["turnWaitTime"]

    def perform(self, connector, reevaluate=False):
        # if we have loked to long in the same direktion without seeing we want to walk
        if time.time() - self.start_time > self.turn_wait_time:
            # reset the timer
            self.start_time = time.time()
            return self.push(PlainWalkAction,
                             [[0, -2, 0, 5]])
        else:
            # Just Search
            return self.push(StopAndSearch)

            # todo run to the middle point of the field after a long while
