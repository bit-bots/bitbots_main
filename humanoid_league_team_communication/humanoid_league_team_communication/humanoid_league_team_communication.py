#!/usr/bin/env python3
import math
import socket
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rospkg
import struct

import tf2_ros
import transforms3d
from rclpy.time import Time
from rclpy.timer import Timer
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from humanoid_league_msgs.msg import GameState, TeamData, ObstacleRelativeArray, ObstacleRelative, Strategy, \
    PoseWithCertaintyStamped
from tf2_geometry_msgs import PointStamped, PoseStamped

import robocup_extension_pb2


class HumanoidLeagueTeamCommunication:
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path("humanoid_league_team_communication")
        self.socket = None
        self.node = Node("team_comm")
        self.node.get_logger().info("Initializing humanoid_league_team_communication...", logger_name="team_comm")

        self.node.declare_parameter('bot_id', 1)
        self.node.declare_parameter('team_id', 1)
        self.node.declare_parameter('map_frame', 'map')
        self.node.declare_parameter('target_host', '127.0.0.1')
        self.node.declare_parameter('target_port', 9999)
        self.node.declare_parameter('receive_port', 9998)
        self.node.declare_parameter('local_target_ports', [])
        self.node.declare_parameter('rate', 1)
        self.node.declare_parameter('life_time', 1)
        self.node.declare_parameter('avg_walking_speed', 0.0)
        self.player_id = self.node.get_parameter('bot_id').get_parameter_value().integer_value
        self.team_id = self.node.get_parameter('team_id').get_parameter_value().integer_value
        self.map_frame = self.node.get_parameter('map_frame').get_parameter_value().string_value
        self.target_host = self.node.get_parameter('target_host').get_parameter_value().string_value
        self.local_target_ports = self.node.get_parameter(
            'local_target_ports').get_parameter_value().integer_array_value
        self.target_port = self.node.get_parameter('target_port').get_parameter_value().integer_value
        self.receive_port = self.node.get_parameter('receive_port').get_parameter_value().integer_value
        self.rate = self.node.get_parameter('rate').get_parameter_value().integer_value
        self.lifetime = self.node.get_parameter('life_time').get_parameter_value().integer_value
        self.avg_walking_speed = self.node.get_parameter('avg_walking_speed').get_parameter_value().double_value

        self.topics = self.node.get_parameters_by_prefix('topics')

        if self.target_host == '127.0.0.1':
            # local mode, bind to port depending on bot id and team id
            self.target_ports = [port + 10 * self.team_id for port in self.local_target_ports]
            self.receive_port = self.target_ports[self.player_id - 1]
        else:
            self.target_ports = [self.target_port]
            self.receive_port = self.receive_port

        self.create_publishers()
        self.create_subscribers()

        self.gamestate = None  # type: GameState
        self.pose = None  # type: PoseWithCovarianceStamped
        self.cmd_vel = None  # type: Twist
        self.cmd_vel_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.ball = None  # type: Optional[PointStamped]
        self.ball_vel = (0, 0, 0)
        self.ball_covariance = None
        self.strategy = None  # type: Strategy
        self.strategy_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.time_to_ball = None
        self.time_to_ball_time = Time(nanoseconds=0, clock_type=self.node.get_clock().clock_type)
        self.obstacles = None  # type: ObstacleRelativeArray
        self.move_base_goal = None  # type: PoseStamped

        # Protobuf <-> ROS Message mappings
        self.team_mapping = (
            (robocup_extension_pb2.Team.UNKNOWN_TEAM, ObstacleRelative.ROBOT_UNDEFINED),
            (robocup_extension_pb2.Team.BLUE, ObstacleRelative.ROBOT_CYAN),
            (robocup_extension_pb2.Team.RED, ObstacleRelative.ROBOT_MAGENTA)
        )
        self.role_mapping = (
            (robocup_extension_pb2.Role.ROLE_UNDEFINED, Strategy.ROLE_UNDEFINED),
            (robocup_extension_pb2.Role.ROLE_IDLING, Strategy.ROLE_IDLING),
            (robocup_extension_pb2.Role.ROLE_OTHER, Strategy.ROLE_OTHER),
            (robocup_extension_pb2.Role.ROLE_STRIKER, Strategy.ROLE_STRIKER),
            (robocup_extension_pb2.Role.ROLE_SUPPORTER, Strategy.ROLE_SUPPORTER),
            (robocup_extension_pb2.Role.ROLE_DEFENDER, Strategy.ROLE_DEFENDER),
            (robocup_extension_pb2.Role.ROLE_GOALIE, Strategy.ROLE_GOALIE),
        )
        self.action_mapping = (
            (robocup_extension_pb2.Action.ACTION_UNDEFINED, Strategy.ACTION_UNDEFINED),
            (robocup_extension_pb2.Action.ACTION_POSITIONING, Strategy.ACTION_POSITIONING),
            (robocup_extension_pb2.Action.ACTION_GOING_TO_BALL, Strategy.ACTION_GOING_TO_BALL),
            (robocup_extension_pb2.Action.ACTION_TRYING_TO_SCORE, Strategy.ACTION_TRYING_TO_SCORE),
            (robocup_extension_pb2.Action.ACTION_WAITING, Strategy.ACTION_WAITING),
            (robocup_extension_pb2.Action.ACTION_KICKING, Strategy.ACTION_KICKING),
            (robocup_extension_pb2.Action.ACTION_SEARCHING, Strategy.ACTION_SEARCHING),
            (robocup_extension_pb2.Action.ACTION_LOCALIZING, Strategy.ACTION_LOCALIZING),
        )
        self.side_mapping = (
            (robocup_extension_pb2.OffensiveSide.SIDE_UNDEFINED, Strategy.SIDE_UNDEFINED),
            (robocup_extension_pb2.OffensiveSide.SIDE_LEFT, Strategy.SIDE_LEFT),
            (robocup_extension_pb2.OffensiveSide.SIDE_MIDDLE, Strategy.SIDE_MIDDLE),
            (robocup_extension_pb2.OffensiveSide.SIDE_RIGHT, Strategy.SIDE_RIGHT),
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # we will try multiple times till we manage to get a connection
        while rclpy.ok() and self.socket is None:
            self.socket = self.get_connection()
            self.node.get_clock().sleep_for(Duration(seconds=1))
            rclpy.spin_once(self.node)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.node.create_timer(1 / self.rate, self.send_message)
        self.receive_forever()

    def create_publishers(self):
        self.pub_team_data = self.node.create_publisher(TeamData, self.topics['team_data_topic'], 1)

    def create_subscribers(self):
        self.node.create_subscription(GameState, self.topics['gamestate_topic'], self.gamestate_cb, 1)
        self.node.create_subscription(PoseWithCovarianceStamped, self.topics['pose_topic'], self.pose_cb, 1)
        self.node.create_subscription(Twist, self.topics['cmd_vel_topic'], self.cmd_vel_cb, 1)
        self.node.create_subscription(PoseWithCovarianceStamped, self.topics['ball_topic'], self.ball_cb, 1)
        self.node.create_subscription(TwistWithCovarianceStamped, self.topics['ball_velocity_topic'], self.ball_vel_cb,
                                      1)
        self.node.create_subscription(Strategy, self.topics['strategy_topic'], self.strategy_cb, 1)
        self.node.create_subscription(Float32, self.topics['time_to_ball_topic'], self.time_to_ball_cb, 1)
        self.node.create_subscription(ObstacleRelativeArray, self.topics['obstacle_topic'], self.obstacle_cb, 1)
        self.node.create_subscription(PoseStamped, self.topics['move_base_goal_topic'], self.move_base_goal_cb, 1)

    def get_connection(self):
        self.node.get_logger().info(f"Binding to port {self.receive_port}", logger_name="team_comm")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(('0.0.0.0', self.receive_port))
        return sock

    def close_connection(self):
        if self.socket:
            self.socket.close()
            self.node.get_logger().info("Connection closed.", logger_name="team_comm")

    def receive_msg(self):
        return self.socket.recv(1024)

    def gamestate_cb(self, msg):
        self.gamestate = msg

    def pose_cb(self, msg):
        self.pose = msg

    def cmd_vel_cb(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_time = self.node.get_clock().now()

    def strategy_cb(self, msg):
        self.strategy = msg
        self.strategy_time = self.node.get_clock().now()

    def time_to_ball_cb(self, msg):
        self.time_to_ball = msg.data
        self.time_to_ball_time = self.node.get_clock().now()

    def move_base_goal_cb(self, msg):
        self.move_base_goal = msg

    def obstacle_cb(self, msg):
        self.obstacles = ObstacleRelativeArray(header=msg.header)
        self.obstacles.header.frame_id = self.map_frame
        for obstacle in msg.obstacles:
            # Transform to map
            obstacle_pose = PoseStamped(msg.header, obstacle.pose.pose.pose)
            try:
                obstacle_map = self.tf_buffer.transform(obstacle_pose, self.map_frame,
                                                        timeout=Duration(nanoseconds=0.3e9))
                obstacle.pose.pose.pose = obstacle_map.pose
                self.obstacles.obstacles.append(obstacle)
            except tf2_ros.TransformException:
                self.node.get_logger().error("TeamComm: Could not transform obstacle to map frame")

    def ball_cb(self, msg: PoseWithCovarianceStamped):
        # Transform to map
        ball_point = PointStamped(msg.header, msg.pose.pose.position)
        try:
            self.ball = self.tf_buffer.transform(ball_point, self.map_frame, timeout=Duration(nanoseconds=0.3e9))
            self.ball_covariance = msg.pose.covariance
        except tf2_ros.TransformException:
            self.node.get_logger().error("TeamComm: Could not transform ball to map frame")
            self.ball = None

    def ball_vel_cb(self, msg: TwistWithCovarianceStamped):
        self.ball_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def __del__(self):
        self.close_connection()

    def receive_forever(self):
        while rclpy.ok():
            try:
                msg = self.receive_msg()
            except (struct.error, socket.timeout):
                continue

            if msg:  # Not handle empty messages or None
                self.handle_message(msg)

    def handle_message(self, msg):
        def covariance_proto_to_ros(fmat3, ros_covariance):
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

        def pose_proto_to_ros(robot, pose):
            pose.pose.position.x = robot.position.x
            pose.pose.position.y = robot.position.y

            quat = transforms3d.euler.euler2quat(0.0, 0.0, robot.position.z)  # wxyz -> ros: xyzw
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            if pose.covariance:
                covariance_proto_to_ros(robot.covariance, pose.covariance)

        message = robocup_extension_pb2.Message()
        message.ParseFromString(msg)

        player_id = message.current_pose.player_id
        team_id = message.current_pose.team

        if player_id == self.player_id or team_id != self.team_id:
            # Skip information from ourselves or from the other team
            return

        team_data = TeamData()

        header = Header()
        # The robots' times can differ, therefore use our own time here
        header.stamp = self.node.get_clock().now()
        header.frame_id = self.map_frame

        # Handle timestamp
        ##################
        team_data.header = header

        # Handle robot ID
        #################
        team_data.robot_id = player_id

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

        self.pub_team_data.publish(team_data)

    def send_message(self, event):
        def covariance_ros_to_proto(ros_covariance, fmat3):
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

        message = robocup_extension_pb2.Message()
        now = self.node.get_clock().now()
        message.timestamp.seconds = now.seconds_nanoseconds()[0]
        message.timestamp.nanos = now.seconds_nanoseconds()[1]

        message.current_pose.player_id = self.player_id
        message.current_pose.team = self.team_id

        if self.gamestate and self.node.get_clock().now() - self.gamestate.header.stamp < Duration(
                seconds=self.lifetime):
            if self.gamestate.penalized:
                # If we are penalized, we are not allowed to send team communication
                return
            else:
                message.state = robocup_extension_pb2.State.UNPENALISED
        else:
            message.state = robocup_extension_pb2.State.UNKNOWN_STATE

        if self.pose and self.node.get_clock().now() - self.pose.header.stamp < Duration(seconds=self.lifetime):
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

        if self.cmd_vel and self.node.get_clock().now() - self.cmd_vel_time < Duration(seconds=self.lifetime):
            message.walk_command.x = self.cmd_vel.linear.x
            message.walk_command.y = self.cmd_vel.linear.y
            message.walk_command.z = self.cmd_vel.angular.z

        if self.move_base_goal and self.node.get_clock().now() - self.move_base_goal.header.stamp < Duration(
                seconds=self.lifetime):
            message.target_pose.position.x = self.move_base_goal.pose.position.x
            message.target_pose.position.y = self.move_base_goal.pose.position.y
            q = self.move_base_goal.pose.orientation
            message.target_pose.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

        if self.ball and self.node.get_clock().now() - self.ball.header.stamp < Duration(seconds=self.lifetime):
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

        if self.obstacles and self.node.get_clock().now() - self.obstacles.header.stamp < Duration(
                seconds=self.lifetime):
            for obstacle in self.obstacles.obstacles:  # type: ObstacleRelative
                if obstacle.type in (ObstacleRelative.ROBOT_CYAN,
                                     ObstacleRelative.ROBOT_MAGENTA,
                                     ObstacleRelative.ROBOT_UNDEFINED):
                    robot = robocup_extension_pb2.Robot()
                    robot.player_id = obstacle.playerNumber
                    robot.position.x = obstacle.pose.pose.pose.position.x
                    robot.position.y = obstacle.pose.pose.pose.position.y
                    q = obstacle.pose.pose.pose.orientation
                    robot.position.z = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]
                    team_mapping = dict(((b, a) for a, b in self.team_mapping))
                    robot.team = team_mapping[obstacle.type]
                    message.others.append(robot)
                    message.other_robot_confidence.append(obstacle.pose.confidence)

        if (self.ball and self.node.get_clock().now() - self.ball.header.stamp < Duration(seconds=self.lifetime) and
                self.pose and self.node.get_clock().now() - self.pose.header.stamp < Duration(seconds=self.lifetime)):
            ball_distance = math.sqrt((self.ball.point.x - self.pose.pose.pose.position.x) ** 2 +
                                      (self.ball.point.y - self.pose.pose.pose.position.y) ** 2)
            message.time_to_ball = ball_distance / self.avg_walking_speed

        if self.strategy and self.node.get_clock().now() - self.strategy_time < Duration(seconds=self.lifetime):
            role_mapping = dict(((b, a) for a, b in self.role_mapping))
            message.role = role_mapping[self.strategy.role]

            action_mapping = dict(((b, a) for a, b in self.action_mapping))
            message.action = action_mapping[self.strategy.action]

            side_mapping = dict(((b, a) for a, b in self.side_mapping))
            message.offensive_side = side_mapping[self.strategy.offensive_side]

        if self.time_to_ball and self.node.get_clock().now() - self.time_to_ball_time < Duration(seconds=self.lifetime):
            message.time_to_ball = self.time_to_ball
        else:
            message.time_to_ball = 9999.0

        msg = message.SerializeToString()
        for port in self.target_ports:
            self.node.get_logger().debug(f'Sending to {port} on {self.target_host}', logger_name="team_comm")
            self.socket.sendto(msg, (self.target_host, port))


def main():
    rclpy.init(args=None)
    HumanoidLeagueTeamCommunication()


if __name__ == '__main__':
    main()
