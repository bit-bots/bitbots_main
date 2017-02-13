# -*- coding:utf-8 -*-
"""
PositionInGoal
^^^^^^^^^^^^^^

Let the goalie position inside the goal.

History:
* 06.12.14: Created using code from GoalieBehaviourDynamic by Daniel Speck (Marc Bestmann)
"""
import math
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.util import get_config

from model.connector import Connector


class PositionInGoal(AbstractDecisionModule):
    def __init__(self,  connector: Connector, _):
        super(PositionInGoal, self).__init__(connector)
        config = get_config()
        goal_width = config["field"]["goal-width"]
        max_side_move = config["Behaviour"]["Goalie"]["maxSidewardMovement"]
        self.angle_threshold = config["Behaviour"]["Goalie"]["sidewardMoveAngleThreshold"]
        self.max_side = goal_width * max_side_move

    def perform(self, connector, reevaluate=False):

        ball_angle = connector.filtered_vision_capsule().get_simple_filtered_ball_info().angle
        # todo allgemeinen filter benutzen #todo was bedeutet dieses todo?
        robot_position = connector.world_model_capsule().get_current_position()  # todo

        # is it necessary to move
        if math.fabs(ball_angle) > self.angle_threshold:

            # left or right
            if ball_angle < 0:
                # are we not to far away from the middle of the goal
                if robot_position[1] < self.max_side:
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_LEFT, 3]])
            else:
                if robot_position[1] > self.max_side:
                    return self.push(PlainWalkAction, [
                        [WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_RIGHT, 3]])
