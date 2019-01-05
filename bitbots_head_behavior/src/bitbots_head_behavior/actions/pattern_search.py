import math

import rospy

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class PatternSearch(AbstractActionElement):
    """
    Executes the configured search_pattern repeatingly in order to try and and see as much
    space as possible and hopefully see the ball.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(PatternSearch, self).__init__(blackboard, dsd, parameters)
        self.index = 0
        self.pattern = self.blackboard.config['search_pattern']
        self.threshold = self.blackboard.config['position_reached_threshold']

    def perform(self, reevaluate=False):
        """
        Look at motor_goals from search_pattern and increment index so that we look somewhere else next

        :param reevaluate:  No effect here
        """
        head_pan, head_tilt = self.pattern[int(self.index)]

        # Convert to radians
        head_pan = head_pan / 180.0 * math.pi
        head_tilt = head_tilt / 180.0 * math.pi
        rospy.logdebug("Searching at {}, {}".format(head_pan, head_tilt))

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt)

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()
        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < (self.threshold / 180.0 * math.pi):
            self.index = (self.index + 1) % len(self.pattern)
