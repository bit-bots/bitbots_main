# -*- coding:utf-8 -*-
"""
ThrowOrRaiseArm
^^^^^^^^^^^^^^^

Handling decsision if we throw ourself or if we just raise a arm.

History:
* 29.11.13: Created (Martin Poppinga)
"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.throw import LEFT, RIGHT, MIDDLE, Throw
from bitbots.modules.behaviour.body.actions.raise_arm import RaiseArm
from bitbots.util import get_config
from bitbots.util.speaker import say

config = get_config()["Behaviour"]


class ThrowOrRaiseArm(AbstractDecisionModule):
    """
    Tests if the ball is dangerous (for the goalie)
    """

    def __init__(self, _):
        super(ThrowOrRaiseArm, self).__init__()
        self.stopReEvaluate = False
        self.richtung = MIDDLE
        self.walking = False
        self.ball_ruht = 0
        self.use_both_arms = config["Toggles"]["Goalie"]["useBothArms"]
        self.toggle_goalie_go_fieldie = config["Toggles"]["Goalie"]["goalieGoFieldie"]
        self.toggle_goalie_go_to_ball = config["Toggles"]["Goalie"]["goalieGoToBall"]
        self.go_to_ball_velocity = config["Goalie"]["goToBallVelocity"]
        self.v_throw_direction_distance = config["Goalie"]["vThrowDirectionDistance"]
        self.u_throw_threshold = config["Goalie"]["uThrowThreshold"]

    def get_throw_direction_on_vestimate(self, vestimate):
        if vestimate < -self.v_throw_direction_distance:
            return RIGHT
        elif vestimate > self.v_throw_direction_distance:
            return LEFT
        else:
            return MIDDLE

    def perform(self, connector, reevaluate=False):
        # When reevaluate isnt set, set stopReEvaluate to False
        # (otherwise stopReEvaluate would be true forever, if set once)
        if not reevaluate:
            self.stopReEvaluate = False

        # Get the estimated values from the BallDataInfoFilterModule
        uestimate, vestimate = connector.filtered_vision_capsule().get_uv_estimate()

        # If they are not valid - return - we can't do anything
        if uestimate is None or vestimate is None:
            say("No valid Estimates")  # todo by Marc: better error handling. Why can there be None values?
            return

        self.richtung = self.get_throw_direction_on_vestimate(vestimate)

        if uestimate <= self.u_throw_threshold:
            return self.push(Throw, self.richtung)
        else:
            if self.richtung != connector.blackboard_capsule().get_arm_pos():
                # todo problems with wrong data in capsule after falling robot?
                return self.push(RaiseArm, self.richtung)

                # If that is all ok we can make further stuff happening
                # eg. voting and switching if the estimates of u is close and long time constant

    def get_reevaluate(self):
        return not self.stopReEvaluate
