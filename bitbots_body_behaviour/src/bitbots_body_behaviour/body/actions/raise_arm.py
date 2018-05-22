# -*- coding:utf-8 -*-
"""
RaiseArm
^^^^^^^^

Raising Arms to get ready to throw the goalie.
"""
from bitbots_body_behaviour.body.actions.throw import LEFT, RIGHT, MIDDLE, BOTH_ARMS_HIGH
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class RaiseArm(AbstractActionModule):
    def __init__(self, connector, args):
        super(RaiseArm, self).__init__(connector)
        self.direction = args
        self.use_both_arms = connector.config["Body"]["Toggles"]["Goalie"]["useBothArms"]
        self.both_arms = connector.animation.config["Goalie"]["raiseBothArms"]
        self.left_arm = connector.animation.config["Goalie"]["raiseLeftArm"]
        self.right_arm = connector.animation.config["Goalie"]["raiseRightArm"]
        self.middle_arm = connector.animation.config["Goalie"]["lowerBothArms"]

    def perform(self, connector, reevaluate=False):
        """ These animations are only correcting the arm positions """
        if not connector.animation.is_animation_busy():
            if self.use_both_arms and (self.direction == LEFT or self.direction == RIGHT):
                # raises both arms, if activated in config
                connector.animation.play_animation(self.both_arms)
            elif self.direction == LEFT:
                connector.animation.play_animation(self.left_arm)
            elif self.direction == RIGHT:
                connector.animation.play_animation(self.right_arm)
            elif self.direction == MIDDLE:
                connector.animation.play_animation(self.middle_arm)
            elif self.direction == BOTH_ARMS_HIGH:
                connector.animation.play_animation(self.both_arms)
            else:
                raise ReferenceError("No LEFT,RIGHT or Middle in RaiseArm")
            connector.blackboard.set_arm_pos(self.direction)
            return self.pop()
