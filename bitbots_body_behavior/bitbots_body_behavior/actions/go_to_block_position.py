import math

import numpy as np
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped

from bitbots_blackboard.blackboard import BodyBlackboard


class GoToBlockPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.block_position_goal_offset = self.blackboard.config["block_position_goal_offset"]
        self.block_radius = self.blackboard.config["block_radius_robot"]
        self.left_goalpost_position = [
            -self.blackboard.world_model.field_length / 2,
            self.blackboard.world_model.goal_width / 2,
        ]  # position of left goalpost
        self.right_goalpost_position = [
            -self.blackboard.world_model.field_length / 2,
            -self.blackboard.world_model.goal_width / 2,
        ]  # position of right goalpost

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
        ball_position = self.blackboard.world_model.get_ball_position_xy()

        ball_to_line_distance = ball_position[0] - self.left_goalpost_position[0]

        opening_angle = self._calc_opening_angle(
            ball_to_line_distance, ball_position
        )  # angle of ball to both goalposts
        angle_bisector = opening_angle / 2  # of the angle between the goalpost

        goalie_pos = self._get_robot_pose(
            angle_bisector, ball_to_line_distance, self.left_goalpost_position, self.right_goalpost_position
        )

        goalie_pos = self._cut_goalie_pos(goalie_pos, ball_position)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = float(goalie_pos[0])
        pose_msg.pose.position.y = float(goalie_pos[1])
        pose_msg.pose.orientation.w = 1.0  # TODO: Change orientation

        self.blackboard.pathfinding.publish(pose_msg)

    def _calc_opening_angle(self, ball_to_line, ball_pos):
        """
        Calculates the opening angle of the ball to both goalposts.
        With it we can get the angle bisector, in which we place the robot.
        Args:
            ball_to_line (float): distance of the ball to our goal line

        Returns:
            float: opening angle
        """
        ball_angle_left = np.arctan2(
            ball_pos[1] - self.left_goalpost_position[1], ball_to_line
        )  # angle of ball to left goalpost
        ball_angle_right = np.arctan2(
            ball_pos[1] - self.right_goalpost_position[1], ball_to_line
        )  # angle of ball to right goalpost
        opening_angle_ball = ball_angle_right - ball_angle_left  # Subtract to get the opening angle
        return opening_angle_ball

    def _get_robot_pose(self, angle_bisector, ball_to_line_distance, ball_pos):
        """
        Calculates the position where the robot should be to block the ball.
        The position is the place where the circle of the robot blocking radius is touching both lines
        from the ball to the goalposts. So the goal is completely guarded.

        Args:
            angle_bisector (float): bisector angle of the ball with goalposts
            ball_to_line_distance (float): distance of the ball to the goal line

        Returns:
            list: goalie_pos
        """
        left_goalpost_to_ball = math.dist(self.left_goalpost_position, ball_pos)
        right_goalpost_to_ball = math.dist(self.right_goalpost_position, ball_pos)

        wanted_distance_robot_to_ball = self.block_radius / np.sin(angle_bisector)  # ball obstruction distance of robot
        angle_of_left_goalpost_to_ball = np.arccos(ball_to_line_distance / left_goalpost_to_ball)
        angle_of_right_goalpost_to_ball = np.arccos(ball_to_line_distance / right_goalpost_to_ball)

        if ball_pos[1] > self.blackboard.world_model.goal_width / 2:
            angle_to_bisector = angle_of_left_goalpost_to_ball + angle_bisector  # here left goalpost angle used
            goalie_x = ball_pos[0] - wanted_distance_robot_to_ball * np.cos(angle_to_bisector)
            goalie_y = ball_pos[1] - wanted_distance_robot_to_ball * np.sin(angle_to_bisector)
        elif ball_pos[1] < -self.blackboard.world_model.goal_width / 2:
            angle_to_bisector = angle_of_right_goalpost_to_ball + angle_bisector  # here right goalpost angle used
            goalie_x = ball_pos[0] - wanted_distance_robot_to_ball * np.cos(angle_to_bisector)
            goalie_y = ball_pos[1] + wanted_distance_robot_to_ball * np.sin(angle_to_bisector)
        else:
            angle_to_bisector = angle_of_left_goalpost_to_ball - angle_bisector  # here right goalpost angle used
            goalie_x = ball_pos[0] - wanted_distance_robot_to_ball * np.cos(angle_to_bisector)
            goalie_y = ball_pos[1] + wanted_distance_robot_to_ball * np.sin(angle_to_bisector)
        return [goalie_x, goalie_y]

    def _cut_goalie_pos(self, goalie_pos, ball_pos):
        """
        Cut the goalie position if he is behind the goal or wants to leave the designated
        goal area.

        Args:
            goalie_pos (list): a list which contains [goalie_y, goalie_x] position

        Returns:
            list: goalie_pos
        """
        penalty_area_length = 2.0  # TODO: Look for world_model parameter
        # cut if goalie is behind the goal happens when the ball is too close to the corner
        if goalie_pos[0] < -self.blackboard.world_model.field_length / 2:
            goalie_pos[0] = -self.blackboard.world_model.field_length / 2 + self.block_position_goal_offset
            # instead put goalie to goalpost position
            if ball_pos[1] > 0:
                goalie_pos[1] = self.blackboard.world_model.goal_width / 2
            else:
                goalie_pos[1] = -self.blackboard.world_model.goal_width / 2
        # cut so goalie does not leave goalkeeper area
        goalie_pos[0] = np.clip(
            goalie_pos[0],
            -self.blackboard.world_model.field_length / 2,
            -self.blackboard.world_model.field_length / 2 + penalty_area_length,
        )
        return goalie_pos
