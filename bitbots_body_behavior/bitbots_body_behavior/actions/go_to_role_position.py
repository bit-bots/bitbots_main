import random

from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToRolePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToRolePosition, self).__init__(blackboard, dsd, parameters)
        role_positions = self.blackboard.config['role_positions']
        try:
            if self.blackboard.blackboard.duty == 'goalie':
                generalized_role_position = role_positions[self.blackboard.blackboard.duty]
            elif self.blackboard.blackboard.duty == 'offense':
                kickoff_type = 'active' if self.blackboard.gamestate.has_kickoff() else 'passive'
                if role_positions['pos_number'] == 0:
                    pos_number = 0
                else:
                    # chose position randomly between left and right
                    pos_number = random.randint(1, 2)
                generalized_role_position = role_positions[f'offense_{kickoff_type}_{pos_number}']
            else:
                # players other than the goalie have multiple possible positions
                generalized_role_position = role_positions[f'{self.blackboard.blackboard.duty}_{pos_number}']
        except KeyError:
            raise KeyError('Role position for {} not specified in config'.format(self.blackboard.blackboard.duty))

        # Adapt position to field size
        # TODO know where map frame is located
        self.role_position = [generalized_role_position[0] * self.blackboard.world_model.field_length / 2,
                              generalized_role_position[1] * self.blackboard.world_model.field_width / 2]

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = self.role_position[0]
        pose_msg.pose.position.y = self.role_position[1]
        pose_msg.pose.orientation.w = 1.0

        self.blackboard.pathfinding.publish(pose_msg)
