"""
BallDangerous
^^^^^^^^^^^^^

Check if the ball is dangerous and we need to do something about it or if we have time for other stuff.
"""
import time

from bitbots_common.stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_common.stackmachine.model import Connector
from body.decisions.goalie.goalie_movement import GoalieMovement
from body.decisions.goalie.throw_or_raise_arm import ThrowOrRaiseArm


class BallDangerous(AbstractDecisionModule):
    def perform(self, connector: Connector, reevaluate=False):
        ufiltered = connector.vision.get_ball_relative()[0]

        # We saw the ball so we track it
        connector.blackboard.schedule_ball_tracking()

        if ufiltered < 1500 and time.time() - connector.vision.get_last_seen("Ball") < 2:
            return self.push(ThrowOrRaiseArm)
        else:
            return self.push(GoalieMovement)

    def get_reevaluate(self):
        return True
