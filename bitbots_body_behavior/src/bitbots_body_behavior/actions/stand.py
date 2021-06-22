import random

import rospy
from tf2_geometry_msgs import PoseStamped

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class CancelPathplanning(AbstractActionElement):
    """Only cancel the pathplanning goal without completly stopping the walking"""

    def perform(self, reevaluate=False):
        self.blackboard.pathfinding.cancel_goal()
        self.pop()


class StandAndWait(AbstractActionElement):
    """This keeps walking in place and optionally pops itself after a given time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super(StandAndWait, self).__init__(blackboard, dsd, parameters)
        self.duration = parameters.get('duration', None)

        self.start_time = rospy.Time.now()

    def perform(self, reevaluate=False):
        self.publish_debug_data("duration", self.duration)
        if self.duration is not None and (rospy.Time.now() - self.start_time) >= rospy.Duration(self.duration):
            return self.pop()

        self.blackboard.pathfinding.cancel_goal()


class Stop(StandAndWait):
    """This stops the robot's walking and optionally pops itself after a given time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard.pathfinding.cancel_goal()

    def perform(self, reevaluate=False):
        self.publish_debug_data("duration", self.duration)
        if self.duration is not None and (rospy.Time.now() - self.start_time) >= rospy.Duration(self.duration):
            return self.pop()
        # need to keep publishing this since path planning publishes a few more messages
        self.blackboard.pathfinding.stop_walk()


class StandAndWaitRandom(Stop):
    """This stops the robot's walking for a random amount of time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.min = parameters.get('min', None)
        self.max = parameters.get('max', None)
        self.duration = random.uniform(self.min, self.max)

        self.start_time = rospy.Time.now()
