# -*- coding:utf-8 -*-
"""
BallDangerous
^^^^^^^^^^^^^

Check if the ball is dangerous and we need to do something about it or if we have time for other stuff.
"""
import rospy

from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

from bitbots_body_behaviour.body.decisions.goalie.goalie_movement import GoalieMovement
from bitbots_body_behaviour.body.decisions.goalie.throw_or_raise_arm import ThrowOrRaiseArm
from humanoid_league_msgs.msg import HeadMode


class BallDangerous(AbstractDecisionElement):
    def __init__(self, connector):
        super(BallDangerous, self).__init__(connector)
        self.react_distance = connector.config["Body"]["Goalie"]["reactDistance"]

    def perform(self, connector, reevaluate=False):
        ball_u = connector.world_model.get_ball_position_uv()[0]

        # We saw the ball so we track it
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        if ball_u < self.react_distance and rospy.get_time() - connector.world_model.ball_last_seen() < 2:
            return self.push(ThrowOrRaiseArm)
        else:
            return self.push(GoalieMovement)

    def get_reevaluate(self):
        return True
