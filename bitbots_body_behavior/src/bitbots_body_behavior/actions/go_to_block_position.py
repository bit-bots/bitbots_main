from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped
import rospy


class GoToBlockPosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToBlockPosition, self).__init__(blackboard, dsd, parameters)
        self.block_position_goal_offset = self.blackboard.config['block_position_goal_offset']

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

        goal_position = (0, -self.blackboard.field_length / 2)  # position of the own goal
        ball_position = self.blackboard.world_model.get_ball_position_xy()

        y_delta =
        x_delta =
        gradient = y_delta / x_delta
        goalie_y = self.block_position_goal_offset * gradient

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = -(self.connector.field_width / 2) + self.block_position_goal_offset
        pose_msg.pose.position.y = self._stay_in_front_of_goal(goalie_y)
        pose_msg.pose.orientation.w = 1

        self.blackboard.pathfinding.publish(pose_msg)

    def _stay_in_front_of_goal(self, y):
        # keeps the y-values of the position in between of the goalposts.
        # this ensures, that y is in [-self.blackboard.goal_width / 2, self.blackboard.goal_width / 2].
        return max(-self.blackboard.goal_width / 2, min(self.blackboard.goal_width / 2, y))
