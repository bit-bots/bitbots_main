from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped
import rospy


class GoToDefensePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToDefensePosition, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        # The defense position should be a position between the ball and the own goal.
        # TODO: in the case of two defensive players, they should use an offset,
        #  so that they are not targeting the same position

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

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = (goal_position[0] + ball_position[0]) /2
        pose_msg.pose.position.y = ball_position[1] / 2
        pose_msg.pose.orientation.w = 1

        self.blackboard.pathfinding.publish(pose_msg)
