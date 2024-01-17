from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped

from bitbots_blackboard.blackboard import BodyBlackboard


class GoToRolePosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        role_positions = self.blackboard.config["role_positions"]
        kickoff_type = "active" if self.blackboard.gamestate.has_kickoff() else "passive"
        try:
            if self.blackboard.team_data.role == "goalie":
                generalized_role_position = role_positions[self.blackboard.team_data.role]
            else:
                # players other than the goalie have multiple possible positions
                generalized_role_position = role_positions[self.blackboard.team_data.role][kickoff_type][
                    str(self.blackboard.misc.position_number)
                ]
        except KeyError as e:
            raise KeyError(f"Role position for {self.blackboard.team_data.role} not specified in config") from e

        # Adapt position to field size
        # TODO know where map frame is located
        self.role_position = [
            generalized_role_position[0] * self.blackboard.world_model.field_length / 2,
            generalized_role_position[1] * self.blackboard.world_model.field_width / 2,
        ]

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = self.role_position[0]
        pose_msg.pose.position.y = self.role_position[1]
        pose_msg.pose.orientation.w = 1.0

        self.blackboard.pathfinding.publish(pose_msg)
