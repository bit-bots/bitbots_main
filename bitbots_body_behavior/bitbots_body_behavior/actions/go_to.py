import math

from bitbots_blackboard.blackboard import BodyBlackboard
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_from_euler

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToRelativePosition(AbstractActionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters: dict = None):
        """Go to a position relative to the robot
        :param dsd:

        """
        super(GoToRelativePosition, self).__init__(blackboard, dsd)
        self.point = float(parameters.get('x', 0)), float(parameters.get('y', 0)), float(parameters.get('t', 0))
        self.first = True

    def perform(self, reevaluate=False):
        if self.first:
            self.first = False
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.blackboard.base_footprint_frame

            pose_msg.pose.position.x = self.point[0]
            pose_msg.pose.position.y = self.point[1]
            pose_msg.pose.position.z = 0.0

            x, y, z, w = quaternion_from_euler(0, 0, math.radians(self.point[2]))
            pose_msg.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)

            # To have the object we are going to in front of us, go to a point behind it
            self.blackboard.pathfinding.publish(pose_msg)
            # TODO: this in good
            # waiting until the robot started to walk
            self.blackboard.node.create_rate(4).sleep()
        if not self.blackboard.blackboard.is_currently_walking():
            self.pop()


class GoToAbsolutePosition(AbstractActionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to an absolute position on the field
        :param dsd:

        """
        super(GoToAbsolutePosition, self).__init__(blackboard, dsd)
        self.point = parameters

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        self.blackboard.pathfinding.publish(pose_msg)


class GoToOwnGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the own goal
        :param dsd:

        """
        super(GoToOwnGoal, self).__init__(blackboard, dsd, parameters)
        self.point = (
            self.blackboard.world_model.get_map_based_own_goal_center_xy()[0],
            self.blackboard.world_model.get_map_based_own_goal_center_xy()[1],
            parameters)


class GoToEnemyGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the enemy goal
        :param dsd:

        """
        super(GoToEnemyGoal, self).__init__(blackboard, dsd, parameters)
        self.point = (
            self.blackboard.world_model.get_map_based_opp_goal_center_xy()[0],
            self.blackboard.world_model.get_map_based_opp_goal_center_xy()[1],
            parameters)


class GoToCenterpoint(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the center of the field and look towards the enemy goal
        :param dsd:
        """
        super(GoToCenterpoint, self).__init__(blackboard, dsd, parameters)
        self.point = 0, 0, 0

