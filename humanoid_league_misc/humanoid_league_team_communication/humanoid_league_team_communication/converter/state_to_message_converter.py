import math
from typing import Callable, List, Optional, Tuple

import transforms3d
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from numpy import double
from rclpy.time import Time
from soccer_vision_3d_msgs.msg import Robot, RobotArray

import humanoid_league_team_communication.robocup_extension_pb2 as Proto  # noqa: N812
from bitbots_msgs.msg import GameState, Strategy


class StateToMessageConverter:
    def __init__(self, team_mapping, role_mapping, action_mapping, side_mapping):
        self.team_mapping = team_mapping
        self.role_mapping = role_mapping
        self.action_mapping = action_mapping
        self.side_mapping = side_mapping

    def convert(self, state, message: Proto.Message, is_still_valid_checker: Callable[[Time], bool]) -> Proto.Message:
        def convert_gamestate(gamestate: Optional[GameState], message: Proto.Message):
            if gamestate is not None and is_still_valid_checker(gamestate.header.stamp):
                message.state = Proto.State.PENALISED if gamestate.penalized else Proto.State.UNPENALISED
            else:
                message.state = Proto.State.UNKNOWN_STATE

            return message

        def convert_current_pose(current_pose: Optional[PoseWithCovarianceStamped], message: Proto.Message):
            if current_pose is not None and is_still_valid_checker(current_pose.header.stamp):
                pose_with_covariance = current_pose.pose
                pose = pose_with_covariance.pose

                message.current_pose.position.x = pose.position.x
                message.current_pose.position.y = pose.position.y
                message.current_pose.position.z = self.extract_orientation_yaw_angle(pose.orientation)

                self.convert_to_covariance_matrix(message.current_pose.covariance, pose_with_covariance.covariance)
            else:
                self.default_to_large_covariance(message.current_pose)

            return message

        def convert_walk_command(walk_command: Optional[Twist], walk_command_time: Time, message: Proto.Message):
            if walk_command is not None and is_still_valid_checker(walk_command_time):
                message.walk_command.x = walk_command.linear.x
                message.walk_command.y = walk_command.linear.y
                message.walk_command.z = walk_command.angular.z

            return message

        def convert_target_position(target_position: Optional[PoseStamped], message):
            if target_position is not None and is_still_valid_checker(target_position.header.stamp):
                pose = target_position.pose
                message.target_pose.position.x = pose.position.x
                message.target_pose.position.y = pose.position.y
                message.target_pose.position.z = self.extract_orientation_yaw_angle(pose.orientation)

            return message

        def convert_ball_position(
            ball_position: Optional[PointStamped],
            ball_velocity: Tuple[float, float, float],
            ball_covariance: List[double],
            message,
        ):
            if ball_position is not None and is_still_valid_checker(ball_position.header.stamp):
                message.ball.position.x = ball_position.point.x
                message.ball.position.y = ball_position.point.y
                message.ball.position.z = ball_position.point.z

                message.ball.velocity.x = ball_velocity[0]
                message.ball.velocity.y = ball_velocity[1]
                message.ball.velocity.z = ball_velocity[2]

                self.convert_to_covariance_matrix(message.ball.covariance, ball_covariance)
            else:
                self.default_to_large_covariance(message.ball)

            return message

        def convert_seen_robots(seen_robots: Optional[RobotArray], message: Proto.Message):
            if seen_robots is not None and is_still_valid_checker(seen_robots.header.stamp):
                seen_robot: Robot
                for seen_robot in seen_robots.robots:
                    robot = Proto.Robot()
                    robot.player_id = seen_robot.attributes.player_number

                    pose = seen_robot.bb.center
                    robot.team = self.team_mapping[seen_robot.attributes.team]
                    robot.position.x = pose.position.x
                    robot.position.y = pose.position.y
                    robot.position.z = self.extract_orientation_yaw_angle(pose.orientation)

                    message.others.append(robot)
                    message.other_robot_confidence.append(seen_robot.confidence.confidence)

            return message

        def convert_strategy(strategy: Optional[Strategy], strategy_time: Time, message: Proto.Message):
            if strategy is not None and is_still_valid_checker(strategy_time):
                message.role = self.role_mapping[strategy.role]
                message.action = self.action_mapping[strategy.action]
                message.offensive_side = self.side_mapping[strategy.offensive_side]

            return message

        def are_robot_and_ball_position_valid(
            current_pose: Optional[PoseWithCovarianceStamped], ball_position: Optional[PointStamped]
        ) -> bool:
            return (
                ball_position is not None
                and is_still_valid_checker(ball_position.header.stamp)
                and current_pose is not None
                and is_still_valid_checker(current_pose.header.stamp)
            )

        def calculate_time_to_ball(
            current_pose: PoseWithCovarianceStamped, ball_position: PointStamped, walking_speed: float
        ) -> float:
            pose = current_pose.pose.pose
            ball_distance = math.sqrt(
                (ball_position.point.x - pose.position.x) ** 2 + (ball_position.point.y - pose.position.y) ** 2
            )

            return ball_distance / walking_speed

        def convert_time_to_ball(
            time_to_ball: Optional[float],
            time_to_ball_time: Time,
            ball_position: PointStamped,
            current_pose: PoseWithCovarianceStamped,
            walking_speed: float,
            message: Proto.Message,
        ):
            if time_to_ball is not None and is_still_valid_checker(time_to_ball_time):
                message.time_to_ball = time_to_ball
            elif are_robot_and_ball_position_valid(current_pose, ball_position):
                message.time_to_ball = calculate_time_to_ball(current_pose, ball_position, walking_speed)
            else:
                message.time_to_ball = 9999.0

            return message

        message.current_pose.player_id = state.player_id
        message.current_pose.team = state.team_id

        message = convert_gamestate(state.gamestate, message)
        message = convert_current_pose(state.pose, message)
        message = convert_walk_command(state.cmd_vel, state.cmd_vel_time, message)
        message = convert_target_position(state.move_base_goal, message)
        message = convert_ball_position(state.ball, state.ball_velocity, state.ball_covariance, message)
        message = convert_seen_robots(state.seen_robots, message)
        message = convert_strategy(state.strategy, state.strategy_time, message)
        message = convert_time_to_ball(
            state.time_to_ball, state.time_to_ball_time, state.ball, state.pose, state.avg_walking_speed, message
        )

        return message

    def default_to_large_covariance(self, message_property, value=100):
        # set high covariance to show that we have no clue
        message_property.covariance.x.x = value
        message_property.covariance.y.y = value
        message_property.covariance.z.z = value

    def extract_orientation_yaw_angle(self, quaternion: Quaternion):
        angles = self.convert_to_euler(quaternion)
        theta = angles[2]
        return theta

    def convert_to_euler(self, quaternion: Quaternion):
        return transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])

    def convert_to_covariance_matrix(self, covariance_matrix: Proto.fmat3, row_major_covariance: List[double]):
        # ROS covariance is row-major 36 x float, while protobuf covariance
        # is column-major 9 x float [x, y, θ]
        covariance_matrix.x.x = row_major_covariance[0]
        covariance_matrix.y.x = row_major_covariance[1]
        covariance_matrix.z.x = row_major_covariance[5]
        covariance_matrix.x.y = row_major_covariance[6]
        covariance_matrix.y.y = row_major_covariance[7]
        covariance_matrix.z.y = row_major_covariance[11]
        covariance_matrix.x.z = row_major_covariance[30]
        covariance_matrix.y.z = row_major_covariance[31]
        covariance_matrix.z.z = row_major_covariance[35]
