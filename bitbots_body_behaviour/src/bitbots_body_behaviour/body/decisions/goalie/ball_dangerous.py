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
from humanoid_league_msgs.msg import HeadMode


class BallDangerous(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        ufiltered = connector.personal_model.get_ball_relative()[0]

        # We saw the ball so we track it
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        if ufiltered < 1500 and rospy.get_time() - connector.personal_model.get_last_seen("Ball") < 2:
            return self.push(ThrowOrRaiseArm)
        else:
            return self.push(GoalieMovement)

    def get_reevaluate(self):
        return True
