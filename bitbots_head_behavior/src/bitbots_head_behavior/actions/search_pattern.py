import math

import rospy

from humanoid_league_msgs.msg import HeadMode

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SearchPattern(AbstractActionElement):
    """
    Executes the configured search_pattern repeatingly in order to try and and see as much
    space as possible and hopefully see the ball.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchPattern, self).__init__(blackboard, dsd, parameters)
        
        self.index = 0

        head_mode = self.blackboard.head_capsule.head_mode
        if head_mode == HeadMode.BALL_MODE_PENALTY:
            pattern_config = self.blackboard.config['search_pattern_penalty']
        else:
            pattern_config = self.blackboard.config['search_pattern']

        self.pan_speed = pattern_config['pan_speed']
        self.tilt_speed = pattern_config['tilt_speed']

        # Generate a search pattern with the min/max values from the config. The min/max statements are used to ensure that the values aren't switched in the config. 
        self.pattern = self.blackboard.head_capsule.generate_pattern(pattern_config['scan_lines'],
                                                                     max(pattern_config['pan_max']),
                                                                     min(pattern_config['pan_max']),
                                                                     max(pattern_config['tilt_max']),
                                                                     min(pattern_config['tilt_max']))
        
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

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()
        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < (self.threshold / 180.0 * math.pi):
            self.index = (self.index + 1) % len(self.pattern)
