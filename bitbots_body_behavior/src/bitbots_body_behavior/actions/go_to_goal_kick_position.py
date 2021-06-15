import math

import numpy as np
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PoseStamped
import rospy
from tf.transformations import quaternion_from_euler


class GoToGoalKickPosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Also apply offset from the ready positions to the defense positions
        role_positions = self.blackboard.config['role_positions']
        self.position_number = role_positions['pos_number']
        try:
            generalized_role_position = \
                role_positions[self.blackboard.blackboard.duty][self.position_number]
        except KeyError:
            raise KeyError('Role position for {} not specified in config'.format(self.blackboard.blackboard.duty))

        self.y_offset = generalized_role_position[1] * self.blackboard.world_model.field_width / 2
        # optional parameter which goes into the block position at a certain distance to the ball
        self.mode = parameters.get('mode', None)
        if self.mode is None or self.mode not in ("offense", "defense"):
            rospy.logerr("mode for goal kick not specified")
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
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.blackboard.map_frame

        if self.mode == "offense":
            # position relative to the goal
            x_to_goal = 2
            y_to_goal = self.y_offset
            x = field_length / 2 + x_to_goal
            y = field_width / 2 + y_to_goal
            yaw = 0
        elif self.mode == "defense":
            x_to_goal = 1
            y_to_goal = self.y_offset
            x = field_length / 2 + x_to_goal
            y = field_width / 2 + y_to_goal
            yaw = 0

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

        self.blackboard.pathfinding.publish(pose_msg)
