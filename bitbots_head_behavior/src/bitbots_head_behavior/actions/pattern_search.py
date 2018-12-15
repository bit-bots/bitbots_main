import math

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class PatternSearch(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(PatternSearch, self).__init__(blackboard, dsd, parameters)
        self.index = 0
        self.pattern = self.blackboard.config['search_pattern']

    def perform(self, reevaluate=False):
        head_pan, head_tilt = self.pattern[int(self.index)]
        # Convert to radians
        head_pan = head_pan / 180.0 * math.pi
        head_tilt = head_tilt / 180.0 * math.pi

        rospy.logdebug_throttle_identical(1, f"Searching at {head_pan}, {head_tilt}")
        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, 1.5, 1.5)
        # Increment index
        self.index = (self.index + 0.2) % len(self.pattern)
