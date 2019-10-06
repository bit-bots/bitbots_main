import abc
import math

import rospy

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractSearchPattern(AbstractActionElement):
    """
    Executes the configured search_pattern repeatingly in order to try and and see as much
    space as possible and hopefully see the ball.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractSearchPattern, self).__init__(blackboard, dsd, parameters)

        pattern_config = self.get_search_pattern()

        self.pan_speed = pattern_config['pan_speed']
        self.tilt_speed = pattern_config['tilt_speed']

        # Generate a search pattern with the min/max values from the config.
        # The min/max statements are used to ensure that the values aren't switched in the config.
        self.pattern = self.blackboard.head_capsule.generate_pattern(pattern_config['scan_lines'],
                                                                     max(pattern_config['pan_max']),
                                                                     min(pattern_config['pan_max']),
                                                                     max(pattern_config['tilt_max']),
                                                                     min(pattern_config['tilt_max']))

        self.threshold = self.blackboard.config['position_reached_threshold']

    @abc.abstractmethod
    def get_search_pattern(self):
        """Get the respective search pattern from the child class"""

    def perform(self, reevaluate=False):
        """
        Look at motor_goals from search_pattern and increment index so that we look somewhere else next

        :param reevaluate:  No effect here
        """
        index = self.blackboard.head_capsule.pattern_index % len(self.pattern)

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()

        min_distance_point = (10000, -1, -1, -1)
        for i, point in enumerate(self.pattern):
            point_pan = math.radians(point[0])
            point_tilt = math.radians(point[1])
            distance = math.sqrt((current_head_pan - math.radians(point_pan)) ** 2 + (current_head_tilt - math.radians(point_tilt)) ** 2)

            if distance < min_distance_point[0]:
                min_distance_point = (distance, point_pan, point_tilt, i)

        if not (self.pattern[int(index)][0] == min_distance_point[1] and \
                self.pattern[int(index)][1] == min_distance_point[2]) or not \
                (self.pattern[(int(index) - 1) % len(self.pattern)][0] == min_distance_point[1] and \
                 self.pattern[(int(index) - 1) % len(self.pattern)][1] == min_distance_point[2]):
            index = min_distance_point[3]

        head_pan, head_tilt = self.pattern[int(index)]

        # Convert to radians
        head_pan = math.radians(head_pan)
        head_tilt = math.radians(head_tilt)
        rospy.logdebug("Searching at {}, {}".format(head_pan, head_tilt))

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)

        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < math.radians(self.threshold):
            self.blackboard.head_capsule.pattern_index = index + 1


class BallSearchPattern(AbstractSearchPattern):
    def get_search_pattern(self):
        return self.blackboard.config['search_pattern']


class PenaltySearchPattern(AbstractSearchPattern):
    def get_search_pattern(self):
        return self.blackboard.config['search_pattern_penalty']


class GoalSearchPattern(AbstractSearchPattern):
    def get_search_pattern(self):
        return self.blackboard.config['search_pattern_goal']


class FieldFeaturesSearchPattern(AbstractSearchPattern):
    def get_search_pattern(self):
        return self.blackboard.config['search_pattern_field_features']


class VisualCompassSearchPattern(AbstractSearchPattern):
    """Search for features that are recognized by the visual compass"""
    def get_search_pattern(self):
        return self.blackboard.config['visual_compass_features_pattern']
