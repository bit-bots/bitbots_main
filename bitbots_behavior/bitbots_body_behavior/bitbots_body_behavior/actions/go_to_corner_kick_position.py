import math

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_from_euler

from bitbots_blackboard.blackboard import BodyBlackboard


class GoToCornerKickPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # optional parameter which goes into the block position at a certain distance to the ball
        self.mode = parameters.get("mode", None)
        if self.mode is None or self.mode not in ("striker", "supporter", "others"):
            self.blackboard.node.get_logger().error("mode for corner kick not specified")
            exit()

    def perform(self, reevaluate=False):
        # The defense position should be a position between the ball and the own goal.

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

        ball_position = self.blackboard.world_model.get_ball_position_xy()
        field_length = self.blackboard.world_model.field_length
        field_width = self.blackboard.world_model.field_width

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        # decide if the corner is on the left or right side of our goal
        if ball_position[1] > 0:
            # on the side of the field where y is positive
            sign = 1
        else:
            sign = -1

        if self.mode == "striker":
            # position relative to the corner
            x_to_corner = 0.5
            y_to_corner = 0.5
            x = field_length / 2 + x_to_corner
            y = sign * (field_width / 2 + y_to_corner)
            yaw = sign * (5 * math.tau / 8)
        elif self.mode == "supporter":
            # position relative to the corner
            x_to_corner = -1.5
            y_to_corner = -1.5
            x = field_length / 2 + x_to_corner
            y = sign * (field_width / 2 + y_to_corner)
            yaw = 0
        elif self.mode == "others":
            # use fixed position rather than standing between ball and goal since there is the goal post
            # x dependent on role position
            if self.blackboard.team_data.role == "defense":
                # close to post
                x_from_goal_line = 0.5
            else:
                # offense players further away based on their position number
                x_from_goal_line = 1.5 + self.blackboard.misc.position_number
            x = x_from_goal_line - (field_length / 2)
            # 1 m away on the side line
            y = sign * ((field_width / 2) - 1)
            yaw = sign * (math.tau / 4)

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        quaternion = Quaternion()
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w
        pose_msg.pose.orientation = quaternion

        self.blackboard.pathfinding.publish(pose_msg)
