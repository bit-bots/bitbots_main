from typing import List, Tuple

import transforms3d
from geometry_msgs.msg import PoseWithCovariance
from numpy import double

import bitbots_team_communication.robocup_extension_pb2 as Proto  # noqa: N812
from bitbots_msgs.msg import RobotRelative, RobotRelativeArray, TeamData


class MessageToTeamDataConverter:
    def __init__(self, team_mapping, role_mapping, action_mapping, side_mapping):
        self.team_mapping = team_mapping
        self.role_mapping = role_mapping
        self.action_mapping = action_mapping
        self.side_mapping = side_mapping

    def convert(self, message: Proto.Message, team_data: TeamData) -> TeamData:
        team_data.robot_id = message.current_pose.player_id
        team_data.state = message.state

        team_data.robot_position = self.convert_robot_pose(message.current_pose)
        team_data.ball_absolute = self.convert_ball_pose(message.ball)

        team_data.robots = self.convert_robots(message.others, message.other_robot_confidence)
        team_data.robots.header = team_data.header

        return self.convert_optional_fields(message, team_data)

    def convert_optional_fields(self, message: Proto.Message, team_data: TeamData) -> TeamData:
        if hasattr(message, "time_to_ball"):
            team_data.time_to_position_at_ball = message.time_to_ball

        return self.convert_strategy(message, team_data)

    def convert_strategy(self, message: Proto.Message, team_data: TeamData) -> TeamData:
        if hasattr(message, "role"):
            team_data.strategy.role = self.role_mapping[message.role]
        if hasattr(message, "action"):
            team_data.strategy.action = self.action_mapping[message.action]
        if hasattr(message, "offensive_side"):
            team_data.strategy.offensive_side = self.side_mapping[message.offensive_side]

        return team_data

    def convert_robots(
        self, message_robots: List[Proto.Robot], message_robot_confidence: List[float]
    ) -> RobotRelativeArray:
        relative_robots = RobotRelativeArray()
        for index, robot in enumerate(message_robots):
            robot_relative = RobotRelative(player_number=robot.player_id, type=self.team_mapping[robot.team])
            robot_relative.pose.pose = self.convert_robot_pose(robot)

            if index < len(message_robot_confidence):
                robot_relative.pose.confidence = message_robot_confidence[index]

            relative_robots.robots.append(robot_relative)

        return relative_robots

    def convert_ball_pose(self, message_ball_pose: Proto.Ball) -> PoseWithCovariance:
        ball = PoseWithCovariance()
        ball.pose.position.x = message_ball_pose.position.x
        ball.pose.position.y = message_ball_pose.position.y
        ball.pose.position.z = message_ball_pose.position.z

        if message_ball_pose.covariance:
            self.convert_to_row_major_covariance(ball.covariance, message_ball_pose.covariance)

        return ball

    def convert_robot_pose(self, message_robot_pose: Proto.Robot) -> PoseWithCovariance:
        robot = PoseWithCovariance()
        robot.pose.position.x = message_robot_pose.position.x
        robot.pose.position.y = message_robot_pose.position.y

        quaternion = self.convert_to_quat((0, 0, message_robot_pose.position.z))
        robot.pose.orientation.w = quaternion[0]
        robot.pose.orientation.x = quaternion[1]
        robot.pose.orientation.y = quaternion[2]
        robot.pose.orientation.z = quaternion[3]

        self.convert_to_row_major_covariance(robot.covariance, message_robot_pose.covariance)

        return robot

    def convert_to_quat(self, euler_angles: Tuple[float, float, float]):
        return transforms3d.euler.euler2quat(*euler_angles)

    def convert_to_row_major_covariance(self, row_major_covariance: List[double], covariance_matrix: Proto.fmat3):
        # ROS covariance is row-major 36 x float, while protobuf covariance
        # is column-major 9 x float [x, y, θ]
        row_major_covariance[0] = covariance_matrix.x.x
        row_major_covariance[1] = covariance_matrix.y.x
        row_major_covariance[5] = covariance_matrix.z.x
        row_major_covariance[6] = covariance_matrix.x.y
        row_major_covariance[7] = covariance_matrix.y.y
        row_major_covariance[11] = covariance_matrix.z.y
        row_major_covariance[30] = covariance_matrix.x.z
        row_major_covariance[31] = covariance_matrix.y.z
        row_major_covariance[35] = covariance_matrix.z.z
