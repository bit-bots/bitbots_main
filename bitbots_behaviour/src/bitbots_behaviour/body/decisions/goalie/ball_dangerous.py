# -*- coding:utf-8 -*-
"""
BallDangerous
^^^^^^^^^^^^^

Check if the ball is dangerous and we need to do something about it or if we have time for other stuff.

History:
* 06.12.14: Created (Marc Bestmann)
"""
import time
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.decisions.goalie.goalie_movement import GoalieMovement
from bitbots.modules.behaviour.body.decisions.goalie.throw_or_raise_arm import ThrowOrRaiseArm


class BallDangerous(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        ufiltered = connector.filtered_vision_capsule().get_local_goal_model_ball()[0]

        # We saw the ball so we track it
        connector.blackboard_capsule().schedule_ball_tracking()

        if ufiltered < 1500 and time.time() - connector.raw_vision_capsule().get_last_seen(
                "Ball") < 2:  # TODO intelligentere Entscheidung
            return self.push(ThrowOrRaiseArm)
        else:
            return self.push(GoalieMovement)

    def get_reevaluate(self):
        return True
