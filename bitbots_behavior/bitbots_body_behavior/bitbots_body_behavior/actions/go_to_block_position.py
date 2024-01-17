from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped

from bitbots_blackboard.blackboard import BodyBlackboard


class GoToBlockPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.block_position_goal_offset = self.blackboard.config["block_position_goal_offset"]
        self.block_position_gradient_factor = self.blackboard.config["block_position_gradient_factor"]

    def perform(self, reevaluate=False):
        # The block position should be a position between the ball and the center of the goal
        # and always in front of the goal

        #      y
        #      ^       ______________________
        #      |    M  |          |          |  O
        #      |    Y  |_ -x, y   |   x, y  _|  P
        #      |    G  | |        |        | |  P
        # 0    +    O  | |       ( )       | |  G
        #      |    A  |_|        |        |_|  O
        #      |    L  |  -x,-y   |   x,-y   |  A
        #      |       |__________|__________|  L
        #      |
        #      +------------------+--------------> x
        #                         0

        goal_position = (-self.blackboard.world_model.field_length / 2, 0)  # position of the own goal
        ball_position = self.blackboard.world_model.get_ball_position_xy()

        x_delta = ball_position[0] - goal_position[0]
        y_delta = ball_position[1]  # goal Y-position is always 0
        gradient = float(y_delta) / float(x_delta) * self.block_position_gradient_factor
        goalie_y = self.block_position_goal_offset * gradient

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = float(
            -(self.blackboard.world_model.field_length / 2) + self.block_position_goal_offset
        )
        pose_msg.pose.position.y = float(self._stay_in_front_of_goal(goalie_y))
        pose_msg.pose.orientation.w = 1.0

        self.blackboard.pathfinding.publish(pose_msg)

    def _stay_in_front_of_goal(self, y):
        # keeps the y-values of the position in between of the goalposts.
        # this ensures, that y is in [-self.blackboard.world_model.goal_width / 2, self.blackboard.world_model.goal_width / 2].
        return max(-self.blackboard.world_model.goal_width / 2, min(self.blackboard.world_model.goal_width / 2, y))
