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
        """
        Executes a defined head pattern.

        :param parameters['init']: Set if the pattern starts at 'ZERO' or the 'NEAREST' position
        """
        super(AbstractSearchPattern, self).__init__(blackboard, dsd, parameters)

        # Assert that a valid direction was supplied
        assert parameters is not None, 'No init specified parameter in parameters (key="init")'
        assert 'init' in parameters, 'No init parameter specified in parameters (key="init")'

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

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()

        # Init index
        if parameters['init'] == "NEAREST":
            self.index = 0

        # Set to the nearest position if wanted
        if parameters['init'] == "ZERO":
            self.index = self._get_near_pattern_position(self.pattern, current_head_pan, current_head_tilt)

    def _get_near_pattern_position(self, pattern, pan, tilt):
        """
        Calculates the nearest position in the head pattern form the current head position.

        :param pattern: The generated head pattern
        :param pan: The current head pan
        :param tilt: The current head tilt
        :return: The index of the nearest pattern position
        """
        # Init the temp distance
        min_distance_point = (10000, -1, -1, -1)
        # Find the smallest distance
        for i, point in enumerate(pattern):
            point_pan = math.radians(point[0])
            point_tilt = math.radians(point[1])
            # Calc the distance
            distance = math.sqrt((pan - math.radians(point_pan)) ** 2 + (tilt - math.radians(point_tilt)) ** 2)
            # Check if distance is smaller than the previous distance
            if distance < min_distance_point[0]:
                # Reset the distance
                min_distance_point = (distance, point_pan, point_tilt, i)
        return min_distance_point[3]

    @abc.abstractmethod
    def get_search_pattern(self):
        """Get the respective search pattern from the child class"""

    def perform(self, reevaluate=False):
        """
        Look at motor_goals from search_pattern and increment index so that we look somewhere else next

        :param reevaluate:  No effect here
        """
        self.index = self.index % len(self.pattern)

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()

        head_pan, head_tilt = self.pattern[int(self.index)]

        # Convert to radians
        head_pan = math.radians(head_pan)
        head_tilt = math.radians(head_tilt)
        rospy.logdebug("Searching at {}, {}".format(head_pan, head_tilt))

        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)

        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < math.radians(self.threshold):
            self.blackboard.head_capsule.pattern_index = self.index + 1


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
