# -*- coding:utf-8 -*-
"""
Throw
^^^^^

Handling throwing of the goalie.
"""

import rospy

from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode

LEFT = "LEFT"
MIDDLE = "MIDDLE"
RIGHT = "RIGHT"
BOTH_ARMS_HIGH = "BOTH"


class Throw(AbstractActionElement):
    """
    Basic version. Throwing to one direction and remembering where we threw ourselves.
    """

    def __init__(self, connector, args):
        super(Throw, self).__init__(connector)
        self.direction = args
        self.initializationTime = rospy.get_time()
        self.played = False
        self.left_animation = connector.animation.config["Goalie"]["throwLeft"]
        self.middle_animation = connector.animation.config["Goalie"]["throwMiddle"]
        self.right_animation = connector.animation.config["Goalie"]["throwRight"]

    def perform(self, connector, reevaluate=False):
        # The head should not move when being thrown
        connector.blackboard.set_head_duty(HeadMode.LOOK_FORWARD)
        if self.direction == LEFT:
            # Returns true if can be performed
            if connector.animation.play_animation(self.left_animation):
                connector.blackboard.set_thrown(LEFT)
                return self.interrupt()
        elif self.direction == RIGHT:
            if connector.animation.play_animation(self.right_animation):
                connector.blackboard.set_thrown(RIGHT)
                connector.blackboard.freeze_till(rospy.get_time() + 4)
                return self.interrupt()
        elif self.direction == MIDDLE:  # MIDDLE
            if connector.animation.play_animation(self.middle_animation):
                connector.blackboard.set_thrown(MIDDLE)
                connector.blackboard.freeze_till(rospy.get_time() + 4)
                return self.interrupt()
        else:
            raise ValueError('%s is not a possible throw direction.' % self.direction)
