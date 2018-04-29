# -*- coding:utf-8 -*-
"""
RaiseArm
^^^^^^^^

Raising Arms to get ready to throw the goalie.
"""
from body.actions.throw import LEFT, RIGHT, MIDDLE, BOTH_ARMS_HIGH
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class RaiseArm(AbstractActionModule):
    def __init__(self, connector, args):
        super(RaiseArm, self).__init__(connector)
        self.richtung = args
        self.use_both_arms = self.connector.config["Behaviour"]["Toggles"]["Goalie"]["useBothArms"]
        self.both_arms = config["animations"]["goalie"]["raiseBoth"]
        self.left_arm = config["animations"]["goalie"]["raiseLeft"]
        self.right_arm = config["animations"]["goalie"]["raiseRight"]
        self.middle_arm = config["animations"]["goalie"]["raiseMiddle"]

    def perform(self, connector, reevaluate=False):
        """ This animations are only correcting the arm positions """
        if not connector.animation.is_animation_busy():
            if self.use_both_arms and (self.richtung == LEFT or self.richtung == RIGHT):
                # raises both arms, if activated in config
                connector.animation.play_animation(self.both_arms)
            elif self.richtung == LEFT:
                connector.animation.play_animation(self.left_arm)
            elif self.richtung == RIGHT:
                connector.animation.play_animation(self.right_arm)
            elif self.richtung == MIDDLE:
                connector.animation.play_animation(self.middle_arm)
            elif self.richtung == BOTH_ARMS_HIGH:
                connector.animation.play_animation(self.both_arms)
            else:
                raise ReferenceError("No LEFT,RIGHT or Middle in RaiseArm")
            connector.blackboard.set_arm_pos(self.richtung)
            return self.pop()
