import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from bitbots_dsd.abstract_action_element import AbstractActionElement


class GoToRolePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        role_positions = self.blackboard.config['role_positions']
        try:
            position_relative = role_positions[self.blackboard.blackboard.duty]
        except KeyError:
            raise KeyError('Role position for {} not specified in config'.format(self.blackboard.blackboard))

        # Adapt position to field size
        # TODO know where map frame is located
        self.role_position = [position_relative[0] * self.blackboard.field_length / 2,
                              position_relative[1] * self.blackboard.field_width / 2]

    def perform(self, reevaluate=False):

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position = Point(self.role_position[0], self.role_position[1], 0)

        pose_msg.pose.orientation = Quaternion(0, 0, 0, 1)

        self.blackboard.pathfinding.call_action(pose_msg)

