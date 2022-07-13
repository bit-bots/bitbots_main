import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class VisualCompassRecord(AbstractActionElement):
    """
    Executes the configured pattern to scan the ground truth for the visual compass.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(VisualCompassRecord, self).__init__(blackboard, dsd, parameters)

        self.index = 0
        self.pan_speed = self.blackboard.config['record_pattern_speed_pan']
        self.tilt_speed = self.blackboard.config['record_pattern_speed_tilt']
        self.interpolation_steps = self.blackboard.config['interpolation_steps']

        # Generate a scan pattern for the left side, with the min/max values from the config. The min/max statements are used to ensure that the values aren't switched in the config.
        self.pattern_left = self.blackboard.head_capsule.generate_pattern(  self.blackboard.config['record_pattern_scan_lines'],
                                                                            max(self.blackboard.config['record_pattern_pan_max_left']),
                                                                            min(self.blackboard.config['record_pattern_pan_max_left']),
                                                                            max(self.blackboard.config['record_pattern_tilt_max']),
                                                                            min(self.blackboard.config['record_pattern_tilt_max']),
                                                                            interpolation_steps=self.interpolation_steps)

        # Generate a scan pattern for the right side, with the min/max values from the config. The min/max statements are used to ensure that the values aren't switched in the config.
        self.pattern_right = self.blackboard.head_capsule.generate_pattern(  self.blackboard.config['record_pattern_scan_lines'],
                                                                            max(self.blackboard.config['record_pattern_pan_max_right']),
                                                                            min(self.blackboard.config['record_pattern_pan_max_right']),
                                                                            max(self.blackboard.config['record_pattern_tilt_max']),
                                                                            min(self.blackboard.config['record_pattern_tilt_max']),
                                                                            interpolation_steps=self.interpolation_steps)

        # Append the two patterns
        self.pattern = self.pattern_left + self.pattern_right
        self.threshold = self.blackboard.config['position_reached_threshold']

    def _notify_visual_compass(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.blackboard.head_capsule.visual_compass_record_trigger.publish(msg)
        self.blackboard.node.get_logger().info("Notify visual compass")

    def perform(self, reevaluate=False):
        """
        Look at motor_goals from search_pattern and increment index so that we look somewhere else next

        :param reevaluate:  No effect here
        """
        head_pan, head_tilt = self.pattern[int(self.index)]

        # Convert to radians
        head_pan = math.radians(head_pan)
        head_tilt = math.radians(head_tilt)
        self.blackboard.node.get_logger().debug("Searching at {}, {}".format(head_pan, head_tilt))
        self.blackboard.head_capsule.send_motor_goals(head_pan, head_tilt, pan_speed=self.pan_speed, tilt_speed=self.tilt_speed)

        current_head_pan, current_head_tilt = self.blackboard.head_capsule.get_head_position()
        distance = math.sqrt((current_head_pan - head_pan) ** 2 + (current_head_tilt - head_tilt) ** 2)

        # Increment index when position is reached
        if distance < math.radians(self.threshold):
            if self.index < len(self.pattern) - 1:
                self.index = (self.index + 1)
                # Notify the visual compass that a new ground truth can be set
                self._notify_visual_compass()
            else:
                self.blackboard.head_capsule.head_mode = 7
                self.index = 0
