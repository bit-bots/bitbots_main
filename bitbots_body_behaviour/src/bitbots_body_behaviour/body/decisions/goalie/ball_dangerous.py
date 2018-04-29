# -*- coding:utf-8 -*-
"""
BallDangerous
^^^^^^^^^^^^^

Check if the ball is dangerous and we need to do something about it or if we have time for other stuff.
"""
import rospy

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

from body.decisions.goalie.goalie_movement import GoalieMovement
from body.decisions.goalie.throw_or_raise_arm import ThrowOrRaiseArm


class BallDangerous(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        ufiltered = connector.vision.get_ball_relative()[0]

        # We saw the ball so we track it
        connector.blackboard.set_head_duty("BALL_MODE")

        if ufiltered < 1500 and rospy.get_time() - connector.vision.get_last_seen("Ball") < 2:
            return self.push(ThrowOrRaiseArm)
        else:
            return self.push(GoalieMovement)

    def get_reevaluate(self):
        return True
