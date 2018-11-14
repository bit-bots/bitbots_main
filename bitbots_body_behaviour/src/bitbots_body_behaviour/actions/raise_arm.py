# -*- coding:utf-8 -*-
"""
RaiseArm
^^^^^^^^

Raising Arms to get ready to throw the goalie.
"""
from bitbots_dsd.abstract_action_element import AbstractActionElement


class RaiseArm(AbstractActionElement):
    def __init__(self, connector, args):
        super(RaiseArm, self).__init__(connector)
        self.direction = args.get("side", "")
        self.use_both_arms = connector.config["Body"]["Toggles"]["Goalie"]["useBothArms"]
        self.both_arms = connector.animation.config["Goalie"]["raiseBothArms"]
        self.left_arm = connector.animation.config["Goalie"]["raiseLeftArm"]
        self.right_arm = connector.animation.config["Goalie"]["raiseRightArm"]
        self.middle_arm = connector.animation.config["Goalie"]["lowerBothArms"]

    def perform(self, blackboard, reevaluate=False):
        """ These animations are only correcting the arm positions """
        if not blackboard.animation.is_animation_busy():
            if self.use_both_arms and (self.direction == "LEFT" or self.direction == "RIGHT"):
                # raises both arms, if activated in config
                blackboard.animation.play_animation(self.both_arms)
            elif self.direction == "LEFT":
                blackboard.animation.play_animation(self.left_arm)
            elif self.direction == "RIGHT":
                blackboard.animation.play_animation(self.right_arm)
            elif self.direction == "MIDDLE":
                blackboard.animation.play_animation(self.middle_arm)
            elif self.direction == "BOTH_ARMS_HIGH":
                blackboard.animation.play_animation(self.both_arms)
            else:
                raise ReferenceError("No LEFT,RIGHT or Middle in RaiseArm")
            blackboard.blackboard.set_arm_pos(self.direction)
            self.pop()
            return