# -*- coding:utf-8 -*-
"""
ThrowOrRaiseArm
^^^^^^^^^^^^^^^

Handling decsision if we throw ourself or if we just raise a arm.

History:
* 29.11.13: Created (Martin Poppinga)
"""
from bitbots_body_behaviour.body.actions.raise_arm import RaiseArm
from bitbots_body_behaviour.body.actions.throw import Throw
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.actions.throw import MIDDLE, LEFT, RIGHT


class ThrowOrRaiseArm(AbstractDecisionModule):
    """
    Tests if the ball is dangerous (for the goalie)
    """

    def __init__(self, connector):
        super(ThrowOrRaiseArm, self).__init__(connector)
        self.stop_reevaluate = False
        self.direction = MIDDLE
        self.walking = False
        self.ball_ruht = 0
        self.use_both_arms = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["useBothArms"]
        self.toggle_goalie_go_fieldie = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["goalieGoFieldie"]
        self.toggle_goalie_go_to_ball = connector.config["Body"]["Behaviour"]["Toggles"]["Goalie"]["goalieGoToBall"]
        self.go_to_ball_velocity = connector.config["Body"]["Behaviour"]["Goalie"]["goToBallVelocity"]
        self.v_throw_direction_distance = connector.config["Body"]["Behaviour"]["Goalie"]["vThrowDirectionDistance"]
        self.u_throw_threshold = connector.config["Body"]["Behaviour"]["Goalie"]["uThrowThreshold"]

    def get_throw_direction(self, ball_v):
        if ball_v < -self.v_throw_direction_distance:
            return RIGHT
        elif ball_v > self.v_throw_direction_distance:
            return LEFT
        else:
            return MIDDLE

    def perform(self, connector, reevaluate=False):
        # When reevaluate is not set, set stop_reevaluate to False
        # (otherwise stop_reevaluate would be true forever, if set once)
        if not reevaluate:
            self.stop_reevaluate = False

        # Get the estimated values from the BallDataInfoFilterModule

        ball_u, ball_v = connector.world_model.get_ball_position_uv()

        # If they are not valid - return - we can't do anything
        if ball_u == ball_v == 0:
            connector.speaker.say("No valid Estimates")
            return

        self.direction = self.get_throw_direction(ball_v)

        if ball_u <= self.u_throw_threshold:
            return self.push(Throw, self.direction)
        else:
            if self.direction != connector.blackboard_capsule().get_arm_pos():
                # todo problems with wrong data in capsule after falling robot?
                return self.push(RaiseArm, self.direction)

                # If that is all ok we can make further stuff happening
                # eg. voting and switching if the estimates of u is close and long time constant

    def get_reevaluate(self):
        return not self.stop_reevaluate
