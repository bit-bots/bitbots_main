import math

from bitbots_dsd.abstract_action_element import AbstractActionElement


class SearchBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchBall, self).__init__(blackboard, dsd, parameters)
        self.index = 0
        self.pattern = self.blackboard.config['search_pattern']

    def perform(self, reevaluate=False):
        head_pan, head_tilt = self.pattern[self.index]
        # Convert to radians
        head_pan = head_pan / 180 * math.pi
        head_tilt = head_tilt / 180 * math.pi
        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt)
        # Increment index
        self.index = (self.index + 1) % len(self.pattern)
