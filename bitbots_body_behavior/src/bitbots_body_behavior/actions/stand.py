import random

import rospy
from tf2_geometry_msgs import PoseStamped

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Stop(AbstractActionElement):
    """ This stops the robot's walking and pops itself when the robot stands """

    def perform(self, reevaluate=False):
        self.blackboard.pathfinding.cancel_goal()
        self.blackboard.pathfinding.stop_walk()
        self.pop()


class CancelPathplanning(AbstractActionElement):
    """Only cancel the pathplanning goal without completly stopping the walking"""

    def perform(self, reevaluate=False):
        self.blackboard.pathfinding.cancel_goal()
        self.pop()


class StandAndWait(AbstractActionElement):
    """ This stops the robots walking and keeps standing """

    def __init__(self, blackboard, dsd, parameters=None):
        super(StandAndWait, self).__init__(blackboard, dsd, parameters)
        self.duration = parameters.get('duration', None)

        self.start_time = rospy.Time.now()

    def perform(self, reevaluate=False):
        self.publish_debug_data("duration", self.duration)
        if self.duration is not None and \
                (rospy.Time.now() - self.start_time) >= rospy.Duration(self.duration):
            return self.pop()

        self.blackboard.pathfinding.cancel_goal()


class StandAndWaitRandom(StandAndWait):
    """ This stops the robots walking and keeps standing """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.min = parameters.get('min', None)
        self.max = parameters.get('max', None)
        self.duration = random.uniform(self.min, self.max)

        self.start_time = rospy.Time.now()
