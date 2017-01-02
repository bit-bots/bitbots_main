# -*- coding:utf-8 -*-
"""
TurnToAbsoluteDirection
^^^^^^^^^^^^^^

Turns the robot to a absolute direction. Its called with direction in degree (0 is opposite ground line) and threshold.
The threshold determines how precise the robot tries to get to this angle. 10 degree threshold will result in up to
10 degree error in each direction, making a range of 20 degree for the robot position.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule


class TurnToAbsoluteDirection(AbstractDecisionModule):
    def __init__(self, args):
        super(TurnToAbsoluteDirection, self).__init__()
        self.goal_direction = args[0]
        self.threshold = args[1]

    def perform(self, connector, reevaluate=False):

        current_direction = connector.world_model_capsule().get_direction()

        if abs(self.goal_direction - current_direction) < self.threshold:
            # we reached the direction exact enough
            connector.walking_capsule().stop_walking()
            return self.pop()
        else:
            if current_direction < self.goal_direction:
                connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_LEFT,
                                                          WalkingCapsule.ZERO)
            else:
                connector.walking_capsule().start_walking(WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT,
                                                          WalkingCapsule.ZERO)
