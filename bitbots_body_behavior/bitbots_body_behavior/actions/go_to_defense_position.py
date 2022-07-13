import math

import numpy as np
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_from_euler


class GoToDefensePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoToDefensePosition, self).__init__(blackboard, dsd, parameters)

        # Also apply offset from the ready positions to the defense positions
        role_positions = self.blackboard.config['role_positions']
        self.position_number = role_positions['pos_number']
        try:
            generalized_role_position = \
                role_positions['defense'][self.position_number]
        except KeyError:
            raise KeyError('Role position for {} not specified in config'.format(self.blackboard.blackboard.duty))

        self.y_offset = generalized_role_position[1] * self.blackboard.world_model.field_width / 2
        # optional parameter which goes into the block position at a certain distance to the ball
        self.mode = parameters.get('mode', None)

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

        goal_position = (-self.blackboard.world_model.field_length / 2, 0)  # position of the own goal
        ball_position = self.blackboard.world_model.get_ball_position_xy()
        robot_x, robot_y, _ = np.array(self.blackboard.world_model.get_current_position())
        our_pose = [robot_x, robot_y]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        if self.mode == "freekick_first":
            vector_ball_to_goal = np.array(goal_position) - np.array(ball_position)
            # pos between ball and goal but 1.5m away from ball
            defense_pos = vector_ball_to_goal / np.linalg.norm(vector_ball_to_goal) * 1.5 + np.array(ball_position)
            pose_msg.pose.position.x = defense_pos[0]
            pose_msg.pose.position.y = defense_pos[1]
            yaw = math.atan(-vector_ball_to_goal[1] / -vector_ball_to_goal[0])
            pose_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        elif self.mode == "freekick_second":
            vector_ball_to_goal = np.array(goal_position) - np.array(ball_position)
            # pos between ball and goal but 1.5m away from ball and 1m to the side which is closer to us
            defense_pos = vector_ball_to_goal / np.linalg.norm(vector_ball_to_goal) * 1.5 + np.array(ball_position)
            yaw = math.atan(-vector_ball_to_goal[1] / -vector_ball_to_goal[0])

            # decide on side that is closer
            pos_1 = np.array([defense_pos[0] + math.sin(yaw) * 1, defense_pos[1] + math.cos(yaw) * 1])
            pos_2 = np.array([defense_pos[0] - math.sin(yaw) * 1, defense_pos[1] - math.cos(yaw) * 1])
            distance_1 = np.linalg.norm(our_pose - pos_1)
            distance_2 = np.linalg.norm(our_pose - pos_2)

            if distance_1 < distance_2:
                pose_msg.pose.position.x = pos_1[0]
                pose_msg.pose.position.y = pos_1[1]
            else:
                pose_msg.pose.position.x = pos_2[0]
                pose_msg.pose.position.y = pos_2[1]
            pose_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        else:
            # center point between ball and own goal
            pose_msg.pose.position.x = (goal_position[0] + ball_position[0]) / 2
            pose_msg.pose.position.y = ball_position[1] / 2 + self.y_offset
            pose_msg.pose.orientation.w = 1

        self.blackboard.pathfinding.publish(pose_msg)
