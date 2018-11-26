# -*- coding:utf-8 -*-
"""
Throw
^^^^^

Handling throwing of the goalie.
"""

import rospy

from bitbots_dsd.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode

LEFT = "LEFT"
MIDDLE = "MIDDLE"
RIGHT = "RIGHT"
BOTH_ARMS_HIGH = "BOTH"


class Throw(AbstractActionElement):
    """
    Basic version. Throwing to one direction and remembering where we threw ourselves.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(Throw, self).__init__(blackboard, dsd)
        self.direction = parameters
        self.initializationTime = rospy.get_time()
        self.played = False
        self.left_animation = blackboard.animation.config["Goalie"]["throwLeft"]
        self.middle_animation = blackboard.animation.config["Goalie"]["throwMiddle"]
        self.right_animation = blackboard.animation.config["Goalie"]["throwRight"]

    def perform(self, reevaluate=False):
        # The head should not move when being thrown
        self.blackboard.blackboard.set_head_duty(HeadMode.LOOK_FORWARD)
        if self.direction == LEFT:
            # Returns true if can be performed
            if self.blackboard.animation.play_animation(self.left_animation):
                self.blackboard.blackboard.set_thrown(LEFT)
                return self.interrupt()
        elif self.direction == RIGHT:
            if self.blackboard.animation.play_animation(self.right_animation):
                self.blackboard.blackboard.set_thrown(RIGHT)
                self.blackboard.blackboard.freeze_till(rospy.get_time() + 4)
                return self.interrupt()
        elif self.direction == MIDDLE:  # MIDDLE
            if self.blackboard.animation.play_animation(self.middle_animation):
                self.blackboard.blackboard.set_thrown(MIDDLE)
                self.blackboard.blackboard.freeze_till(rospy.get_time() + 4)
                return self.interrupt()
        else:
            raise ValueError('%s is not a possible throw direction.' % self.direction)
