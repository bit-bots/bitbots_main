import random

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

from bitbots_blackboard.blackboard import BodyBlackboard


class CancelPathplanning(AbstractActionElement):
    """Only cancel the pathplanning goal without completly stopping the walking"""

    def perform(self, reevaluate=False):
        self.blackboard: BodyBlackboard
        self.blackboard.pathfinding.cancel_goal()
        self.pop()


class WalkInPlace(AbstractActionElement):
    """This keeps walking in place and optionally pops itself after a given time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard
        self.duration = parameters.get("duration", None)

        self.start_time = self.blackboard.node.get_clock().now()

        # Cancel the path planning if it is running
        self.blackboard.pathfinding.cancel_goal()

    def perform(self, reevaluate=False):
        self.publish_debug_data("duration", self.duration)
        if self.duration is not None and (self.blackboard.node.get_clock().now() - self.start_time) >= Duration(
            seconds=self.duration
        ):
            return self.pop()

        self.blackboard.pathfinding.direct_cmd_vel_pub.publish(Twist())


class Stand(WalkInPlace):
    """This stops the robot's walking and optionally pops itself after a given time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        # Cancel the path planning if it is running
        self.blackboard.pathfinding.cancel_goal()

    def perform(self, reevaluate=False):
        self.publish_debug_data("duration", self.duration)
        if self.duration is not None and (self.blackboard.node.get_clock().now() - self.start_time) >= Duration(
            seconds=self.duration
        ):
            return self.pop()
        # need to keep publishing this since path planning publishes a few more messages
        self.blackboard.pathfinding.stop_walk()


class StandAndWaitRandom(Stand):
    """This stops the robot's walking for a random amount of time"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.min = parameters.get("min", None)
        self.max = parameters.get("max", None)
        self.duration = random.uniform(self.min, self.max)

        self.start_time = self.blackboard.node.get_clock().now()
