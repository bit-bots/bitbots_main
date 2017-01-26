"""
Throw
^^^^^

Handling throwing of the goalie.
"""

import time

from stackmachine.abstract_action_module import AbstractActionModule

LEFT = "LEFT"
MIDDLE = "MIDDLE"
RIGHT = "RIGHT"
BOTH_ARMS_HIGH = "BOTH"


class Throw(AbstractActionModule):
    """
    Basic version. Throwing to one direction and remembering where we trowed ourselfs.
    """

    def __init__(self, args):
        super(Throw, self).__init__()
        self.richtung = args
        self.initializationTime = time.time()
        self.played = False
        self.left_animation = config["animations"]["goalie"]["throw_left"]
        self.middle_animation = config["animations"]["goalie"]["throw_middle"]
        self.right_animation = config["animations"]["goalie"]["throw_right"]

    def perform(self, connector, reevaluate=False):

        connector.blackboard.cancel_ball_tracking()
        if self.richtung == LEFT:
            # Returns true if can be performed
            if connector.animation.play_animation(self.left_animation):
                connector.blackboard.set_thrown(LEFT)
                return self.interrupt()
        elif self.richtung == RIGHT:
            if connector.animation.play_animation(self.right_animation):
                connector.blackboard.set_thrown(RIGHT)
                connector.blackboard.freeze_till(time.time() + 4)
                return self.interrupt()
        elif self.richtung == MIDDLE:  # MIDDLE
            if connector.animation.play_animation(self.middle_animation):
                connector.blackboard.set_thrown(MIDDLE)
                connector.blackboard.freeze_till(time.time() + 4)
                return self.interrupt()
        else:
            raise ValueError('%s is not a possible throw direction.' % self.richtung)
