import rospy
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToRolePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToRolePosition, self).__init__(blackboard, dsd, parameters)
        role_positions = self.blackboard.config['role_positions']
        try:
            generalized_role_position = role_positions[self.blackboard.blackboard.duty]
        except KeyError:
            raise KeyError('Role position for {} not specified in config'.format(self.blackboard.blackboard.duty))

        # Adapt position to field size
        # TODO know where map frame is located
        self.role_position = [generalized_role_position[0] * self.blackboard.field_length / 2,
                              generalized_role_position[1] * self.blackboard.field_width / 2]

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.role_position[0]
        pose_msg.pose.position.y = self.role_position[1]
        pose_msg.pose.orientation.w = 1

        self.blackboard.pathfinding.publish(pose_msg)
