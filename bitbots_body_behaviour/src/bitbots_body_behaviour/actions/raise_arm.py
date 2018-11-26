# -*- coding:utf-8 -*-
"""
RaiseArm
^^^^^^^^

Raising Arms to get ready to throw the goalie.
"""
from bitbots_dsd.abstract_action_element import AbstractActionElement


class RaiseArm(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(RaiseArm, self).__init__(blackboard, dsd)
        self.direction = parameters.get("side", "")
        self.use_both_arms = blackboard.config["Body"]["Toggles"]["Goalie"]["useBothArms"]
        self.both_arms = blackboard.animation.config["Goalie"]["raiseBothArms"]
        self.left_arm = blackboard.animation.config["Goalie"]["raiseLeftArm"]
        self.right_arm = blackboard.animation.config["Goalie"]["raiseRightArm"]
        self.middle_arm = blackboard.animation.config["Goalie"]["lowerBothArms"]

    def perform(self, reevaluate=False):
        """ These animations are only correcting the arm positions """
        if not self.blackboard.animation.is_animation_busy():
            if self.use_both_arms and (self.direction == "LEFT" or self.direction == "RIGHT"):
                # raises both arms, if activated in config
                self.blackboard.animation.play_animation(self.both_arms)
            elif self.direction == "LEFT":
                self.blackboard.animation.play_animation(self.left_arm)
            elif self.direction == "RIGHT":
                self.blackboard.animation.play_animation(self.right_arm)
            elif self.direction == "MIDDLE":
                self.blackboard.animation.play_animation(self.middle_arm)
            elif self.direction == "BOTH_ARMS_HIGH":
                self.blackboard.animation.play_animation(self.both_arms)
            else:
                raise ReferenceError("No LEFT,RIGHT or Middle in RaiseArm")
            self.blackboard.blackboard.set_arm_pos(self.direction)
            self.pop()
            return
