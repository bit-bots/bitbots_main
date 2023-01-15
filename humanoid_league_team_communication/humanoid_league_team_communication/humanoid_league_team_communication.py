#!/usr/bin/env python3

import math
import socket
import struct
import threading
from numpy import double
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
import transforms3d
from rclpy.time import Time
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from humanoid_league_msgs.msg import GameState, TeamData, ObstacleRelativeArray, ObstacleRelative, Strategy
from tf2_geometry_msgs import PointStamped, PoseStamped
from bitbots_utils.utils import get_parameter_dict, get_parameters_from_other_node
from ament_index_python.packages import get_package_share_directory

from humanoid_league_team_communication.communication import SocketCommunication
from humanoid_league_team_communication.robocup_extension_pb2 import *


class HumanoidLeagueTeamCommunication:

    def __init__(self):
        self._package_path = get_package_share_directory("humanoid_league_team_communication")
        self.node = Node("team_comm", automatically_declare_parameters_from_overrides=True)
        self.logger = self.node.get_logger()

        self.logger.info("Initializing humanoid_league_team_communication...")

        params_blackboard = get_parameters_from_other_node(self.node, "parameter_blackboard", ['bot_id', 'team_id'])
        self.player_id = params_blackboard['bot_id']
        self.team_id = params_blackboard['team_id']

        self.socket_communication = SocketCommunication(self.node, self.logger, self.team_id, self.player_id)

        self.rate = self.node.get_parameter('rate').get_parameter_value().integer_value
        self.lifetime = self.node.get_parameter('lifetime').get_parameter_value().integer_value
        self.avg_walking_speed = self.node.get_parameter('avg_walking_speed').get_parameter_value().double_value

        self.topics = get_parameter_dict(self.node, 'topics')
        self.map_frame = self.node.get_parameter('map_frame').get_parameter_value().string_value

        self.create_publishers()
        self.create_subscribers()

        self.set_state_defaults()

        # Protobuf <-> ROS Message mappings
        self.team_mapping = ((Team.UNKNOWN_TEAM, ObstacleRelative.ROBOT_UNDEFINED),
                             (Team.BLUE, ObstacleRelative.ROBOT_CYAN), (Team.RED, ObstacleRelative.ROBOT_MAGENTA))
        self.role_mapping = (
            (Role.ROLE_UNDEFINED, Strategy.ROLE_UNDEFINED),
            (Role.ROLE_IDLING, Strategy.ROLE_IDLING),
            (Role.ROLE_OTHER, Strategy.ROLE_OTHER),
            (Role.ROLE_STRIKER, Strategy.ROLE_STRIKER),
            (Role.ROLE_SUPPORTER, Strategy.ROLE_SUPPORTER),
            (Role.ROLE_DEFENDER, Strategy.ROLE_DEFENDER),
            (Role.ROLE_GOALIE, Strategy.ROLE_GOALIE),
        )
        self.action_mapping = (
            (Action.ACTION_UNDEFINED, Strategy.ACTION_UNDEFINED),
            (Action.ACTION_POSITIONING, Strategy.ACTION_POSITIONING),
            (Action.ACTION_GOING_TO_BALL, Strategy.ACTION_GOING_TO_BALL),
            (Action.ACTION_TRYING_TO_SCORE, Strategy.ACTION_TRYING_TO_SCORE),
            (Action.ACTION_WAITING, Strategy.ACTION_WAITING),
            (Action.ACTION_KICKING, Strategy.ACTION_KICKING),
            (Action.ACTION_SEARCHING, Strategy.ACTION_SEARCHING),
            (Action.ACTION_LOCALIZING, Strategy.ACTION_LOCALIZING),
        )
        self.side_mapping = (
            (OffensiveSide.SIDE_UNDEFINED, Strategy.SIDE_UNDEFINED),
            (OffensiveSide.SIDE_LEFT, Strategy.SIDE_LEFT),
            (OffensiveSide.SIDE_MIDDLE, Strategy.SIDE_MIDDLE),
            (OffensiveSide.SIDE_RIGHT, Strategy.SIDE_RIGHT),
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.try_to_establish_connection()
        self.run_spin_in_thread()

        self.node.create_timer(1 / self.rate, self.send_message)
        self.receive_forever()

    def run_spin_in_thread(self):
        # Necessary in ROS2, else we are forever stuck receiving messages
        thread = threading.Thread(target=rclpy.spin, args=[self.node], daemon=True)
        thread.start()

    def set_state_defaults(self):
        self.gamestate: GameState = None
        self.pose: PoseWithCovarianceStamped = None
        self.cmd_vel: Twist = None
        self.cmd_vel_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.ball: Optional[PointStamped] = None
        self.ball_velocity: Tuple[float, float, float] = (0, 0, 0)
        self.ball_covariance: List[double] = None
        self.strategy: Strategy = None
        self.strategy_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.time_to_ball: float = None
        self.time_to_ball_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.obstacles: ObstacleRelativeArray = None
        self.move_base_goal: PoseStamped = None

    def try_to_establish_connection(self):
        # we will try multiple times till we manage to get a connection
        while rclpy.ok() and not self.socket_communication.is_setup():
            self.socket_communication.establish_connection()
            self.node.get_clock().sleep_for(Duration(seconds=1))
            rclpy.spin_once(self.node)

    def create_publishers(self):
        self.team_data_publisher = self.node.create_publisher(TeamData, self.topics['team_data_topic'], 1)

    def create_subscribers(self):
        self.node.create_subscription(GameState, self.topics['gamestate_topic'], self.gamestate_cb, 1)
        self.node.create_subscription(PoseWithCovarianceStamped, self.topics['pose_topic'], self.pose_cb, 1)
        self.node.create_subscription(Twist, self.topics['cmd_vel_topic'], self.cmd_vel_cb, 1)
        self.node.create_subscription(PoseWithCovarianceStamped, self.topics['ball_topic'], self.ball_cb, 1)
        self.node.create_subscription(TwistWithCovarianceStamped, self.topics['ball_velocity_topic'],
                                      self.ball_velocity_cb, 1)
        self.node.create_subscription(Strategy, self.topics['strategy_topic'], self.strategy_cb, 1)
        self.node.create_subscription(Float32, self.topics['time_to_ball_topic'], self.time_to_ball_cb, 1)
        self.node.create_subscription(ObstacleRelativeArray, self.topics['obstacle_topic'], self.obstacle_cb, 1)
        self.node.create_subscription(PoseStamped, self.topics['move_base_goal_topic'], self.move_base_goal_cb, 1)

    def gamestate_cb(self, msg: GameState):
        self.gamestate = msg

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel = msg
        self.cmd_vel_time = self.get_current_time()

    def strategy_cb(self, msg: Strategy):
        self.strategy = msg
        self.strategy_time = self.get_current_time()

    def time_to_ball_cb(self, msg: float):
        self.time_to_ball = msg.data
        self.time_to_ball_time = self.get_current_time()

    def move_base_goal_cb(self, msg: PoseStamped):
        self.move_base_goal = msg

    def obstacle_cb(self, msg: ObstacleRelativeArray):

        def transform_to_map(obstacle: ObstacleRelative):
            obstacle_pose = PoseStamped(header=msg.header, pose=obstacle.pose.pose.pose)
            try:
                obstacle_map = self.tf_transform(obstacle_pose)
                obstacle.pose.pose.pose = obstacle_map.pose
                return obstacle
            except tf2_ros.TransformException:
                self.logger.error("TeamComm: Could not transform obstacle to map frame")

        self.obstacles = ObstacleRelativeArray(header=msg.header)
        self.obstacles.header.frame_id = self.map_frame
        self.obstacles.obstacles = list(map(transform_to_map, msg.obstacles))

    def ball_cb(self, msg: PoseWithCovarianceStamped):
        ball_point = PointStamped(header=msg.header, point=msg.pose.pose.position)
        try:
            self.ball = self.tf_transform(ball_point)
            self.ball_covariance = msg.pose.covariance
        except tf2_ros.TransformException:
            self.logger.error("TeamComm: Could not transform ball to map frame")
            self.ball = None

    def ball_velocity_cb(self, msg: TwistWithCovarianceStamped):
        self.ball_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def tf_transform(self, field, timeout_in_ns=0.3e9):
        return self.tf_buffer.transform(field, self.map_frame, timeout=Duration(nanoseconds=timeout_in_ns))

    def receive_forever(self):
        while rclpy.ok():
            try:
                message = self.socket_communication.receive_message()
            except (struct.error, socket.timeout):
                continue

            if message:
                self.handle_message(message)

    def handle_message(self, string_message: bytes):

        def covariance_proto_to_ros(fmat3: fmat3, ros_covariance: List[double]):
            # ROS covariance is row-major 36 x float, protobuf covariance is column-major 9 x float [x, y, θ]
            ros_covariance[0] = fmat3.x.x
            ros_covariance[1] = fmat3.y.x
            ros_covariance[5] = fmat3.z.x
            ros_covariance[6] = fmat3.x.y
            ros_covariance[7] = fmat3.y.y
            ros_covariance[11] = fmat3.z.y
            ros_covariance[30] = fmat3.x.z
            ros_covariance[31] = fmat3.y.z
            ros_covariance[35] = fmat3.z.z

        def pose_proto_to_ros(robot: Robot, pose: PoseWithCovariance):
            pose.pose.position.x = robot.position.x
            pose.pose.position.y = robot.position.y

            quat = transforms3d.euler.euler2quat(0.0, 0.0, robot.position.z)  # wxyz -> ros: xyzw
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            if pose.covariance:
                covariance_proto_to_ros(robot.covariance, pose.covariance)

        message = Message()
        message.ParseFromString(string_message)

        if self.should_message_be_discarded(message):
            return

        team_data = TeamData()

        header = Header()
        # The robots' times can differ, therefore use our own time here
        header.stamp = self.get_current_time()
        header.frame_id = self.map_frame

        # Handle timestamp
        ##################
        team_data.header = header

        # Handle robot ID
        #################
        team_data.robot_id = message.current_pose.player_id

        # Handle state
        ##############
        team_data.state = message.state

        # Handle pose of current player
        ###############################
        pose_proto_to_ros(message.current_pose, team_data.robot_position)

        # Handle ball
        #############
        team_data.ball_absolute.pose.position.x = message.ball.position.x
        team_data.ball_absolute.pose.position.y = message.ball.position.y
        team_data.ball_absolute.pose.position.z = message.ball.position.z

        if message.ball.covariance:
            covariance_proto_to_ros(message.ball.covariance, team_data.ball_absolute.covariance)

        # Handle obstacles
        ##################
        obstacle_relative_array = ObstacleRelativeArray()
        obstacle_relative_array.header = header

        for index, robot in enumerate(message.others):
            obstacle = ObstacleRelative()

            # Obstacle type
            team_to_obstacle_type = dict(self.team_mapping)
            obstacle.type = team_to_obstacle_type[robot.team]

            obstacle.playerNumber = robot.player_id

            pose_proto_to_ros(robot, obstacle.pose.pose)
            if hasattr(message, "other_robot_confidence") and index < len(message.other_robot_confidence):
                obstacle.pose.confidence = message.other_robot_confidence[index]

            team_data.obstacles.obstacles.append(obstacle)

        # Handle time to position at ball
        #################################
        if hasattr(message, "time_to_ball"):
            team_data.time_to_position_at_ball = message.time_to_ball

        # Handle strategy
        #################
        if hasattr(message, "role"):
            role_mapping = dict(self.role_mapping)
            team_data.strategy.role = role_mapping[message.role]
        if hasattr(message, "action"):
            action_mapping = dict(self.action_mapping)
            team_data.strategy.action = action_mapping[message.action]
        if hasattr(message, "offensive_side"):
            offensive_side_mapping = dict(self.side_mapping)
            team_data.strategy.offensive_side = offensive_side_mapping[message.offensive_side]

        self.team_data_publisher.publish(team_data)

    def send_message(self):

        def covariance_ros_to_proto(ros_covariance: List[double], fmat3: fmat3):
            # ROS covariance is row-major 36 x float, protobuf covariance is column-major 9 x float [x, y, θ]
            fmat3.x.x = ros_covariance[0]
            fmat3.y.x = ros_covariance[1]
            fmat3.z.x = ros_covariance[5]
            fmat3.x.y = ros_covariance[6]
            fmat3.y.y = ros_covariance[7]
            fmat3.z.y = ros_covariance[11]
            fmat3.x.z = ros_covariance[30]
            fmat3.y.z = ros_covariance[31]
            fmat3.z.z = ros_covariance[35]

        message = Message()
        now = self.get_current_time()
        message.timestamp.seconds = now.seconds_nanoseconds()[0]
        message.timestamp.nanos = now.seconds_nanoseconds()[1]

        message.current_pose.player_id = self.player_id
        message.current_pose.team = self.team_id

        if self.gamestate and now - self.gamestate.header.stamp < Duration(seconds=self.lifetime):
            if self.gamestate.penalized:
                # If we are penalized, we are not allowed to send team communication
                return
            else:
                message.state = State.UNPENALISED
        else:
            message.state = State.UNKNOWN_STATE

        if self.pose and now - self.pose.header.stamp < Duration(seconds=self.lifetime):
            message.current_pose.position.x = self.pose.pose.pose.position.x
            message.current_pose.position.y = self.pose.pose.pose.position.y
            q = self.pose.pose.pose.orientation
            # z is theta
            message.current_pose.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
            covariance_ros_to_proto(self.pose.pose.covariance, message.current_pose.covariance)
        else:
            # set high covariance to show that we have no clue
            message.current_pose.covariance.x.x = 100
            message.current_pose.covariance.y.y = 100
            message.current_pose.covariance.z.z = 100

        if self.cmd_vel and now - self.cmd_vel_time < Duration(seconds=self.lifetime):
            message.walk_command.x = self.cmd_vel.linear.x
            message.walk_command.y = self.cmd_vel.linear.y
            message.walk_command.z = self.cmd_vel.angular.z

        if self.move_base_goal and now - self.move_base_goal.header.stamp < Duration(seconds=self.lifetime):
            message.target_pose.position.x = self.move_base_goal.pose.position.x
            message.target_pose.position.y = self.move_base_goal.pose.position.y
            q = self.move_base_goal.pose.orientation
            message.target_pose.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

        if self.ball and now - self.ball.header.stamp < Duration(seconds=self.lifetime):
            message.ball.position.x = self.ball.point.x
            message.ball.position.y = self.ball.point.y
            message.ball.position.z = self.ball.point.z
            message.ball.velocity.x = self.ball_vel[0]
            message.ball.velocity.y = self.ball_vel[1]
            message.ball.velocity.z = self.ball_vel[2]
            covariance_ros_to_proto(self.ball_covariance, message.ball.covariance)
        else:
            # set high covariance to show that we have no clue
            message.ball.covariance.x.x = 100
            message.ball.covariance.y.y = 100
            message.ball.covariance.z.z = 100

        if self.obstacles and now - self.obstacles.header.stamp < Duration(seconds=self.lifetime):
            for obstacle in self.obstacles.obstacles:
                obstacle: ObstacleRelative
                if obstacle.type in (ObstacleRelative.ROBOT_CYAN, ObstacleRelative.ROBOT_MAGENTA,
                                     ObstacleRelative.ROBOT_UNDEFINED):
                    robot = Robot()
                    robot.player_id = obstacle.playerNumber
                    robot.position.x = obstacle.pose.pose.pose.position.x
                    robot.position.y = obstacle.pose.pose.pose.position.y
                    q = obstacle.pose.pose.pose.orientation
                    robot.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
                    team_mapping = dict(((b, a) for a, b in self.team_mapping))
                    robot.team = team_mapping[obstacle.type]
                    message.others.append(robot)
                    message.other_robot_confidence.append(obstacle.pose.confidence)

        if (self.ball and now - self.ball.header.stamp < Duration(seconds=self.lifetime) and self.pose and
                now - self.pose.header.stamp < Duration(seconds=self.lifetime)):
            ball_distance = math.sqrt((self.ball.point.x - self.pose.pose.pose.position.x)**2 +
                                      (self.ball.point.y - self.pose.pose.pose.position.y)**2)
            message.time_to_ball = ball_distance / self.avg_walking_speed

        if self.strategy and now - self.strategy_time < Duration(seconds=self.lifetime):
            role_mapping = dict(((b, a) for a, b in self.role_mapping))
            message.role = role_mapping[self.strategy.role]

            action_mapping = dict(((b, a) for a, b in self.action_mapping))
            message.action = action_mapping[self.strategy.action]

            side_mapping = dict(((b, a) for a, b in self.side_mapping))
            message.offensive_side = side_mapping[self.strategy.offensive_side]

        if self.time_to_ball and now - self.time_to_ball_time < Duration(seconds=self.lifetime):
            message.time_to_ball = self.time_to_ball
        else:
            message.time_to_ball = 9999.0

        self.socket_communication.send_message(message.SerializeToString())

    def should_message_be_discarded(self, message: Message) -> bool:
        player_id = message.current_pose.player_id
        team_id = message.current_pose.team

        isOwnMessage = player_id == self.player_id
        isMessageFromOpositeTeam = team_id != self.team_id

        return isOwnMessage or isMessageFromOpositeTeam

    def is_robot_allowed_to_send_message(self) -> bool:
        return self.gamestate and not self.gamestate.penalized

    def get_current_time(self) -> Time:
        return self.node.get_clock().now()


def main():
    rclpy.init(args=None)
    HumanoidLeagueTeamCommunication()


if __name__ == '__main__':
    main()
