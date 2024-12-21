#!/usr/bin/env python3

import socket
import struct
import threading
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from bitbots_tf_buffer import Buffer
from bitbots_utils.utils import get_parameter_dict, get_parameters_from_other_node
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from numpy import double
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from soccer_vision_3d_msgs.msg import Robot, RobotArray
from std_msgs.msg import Float32, Header
from tf2_geometry_msgs import PointStamped, PoseStamped
from tf2_ros import TransformException

import bitbots_team_communication.robocup_extension_pb2 as Proto  # noqa: N812
from bitbots_msgs.msg import Strategy, TeamData
from bitbots_team_communication.communication import SocketCommunication
from bitbots_team_communication.converter.robocup_protocol_converter import RobocupProtocolConverter, TeamColor


class TeamCommunication:
    def __init__(self):
        self._package_path = get_package_share_directory("bitbots_team_communication")
        self.node = Node("team_comm", automatically_declare_parameters_from_overrides=True)
        self.logger = self.node.get_logger()

        self.logger.info("Initializing bitbots_team_communication...")
        params_blackboard = get_parameters_from_other_node(
            self.node, "parameter_blackboard", ["bot_id", "team_id", "team_color"]
        )
        self.player_id = params_blackboard["bot_id"]
        self.team_id = params_blackboard["team_id"]
        self.team_color_id = params_blackboard["team_color"]

        self.protocol_converter = RobocupProtocolConverter(TeamColor(self.team_color_id))

        self.logger.info(f"Starting for {self.player_id} in team {self.team_id}...")
        self.socket_communication = SocketCommunication(self.node, self.logger, self.team_id, self.player_id)

        self.rate: int = self.node.get_parameter("rate").value
        self.lifetime: int = self.node.get_parameter("lifetime").value
        self.avg_walking_speed: float = self.node.get_parameter("avg_walking_speed").value

        self.topics = get_parameter_dict(self.node, "topics")
        self.map_frame: str = self.node.get_parameter("map_frame").value

        self.create_publishers()
        self.create_subscribers()

        self.set_state_defaults()

        self.tf_buffer = Buffer(self.node)

        self.run_spin_in_thread()
        self.try_to_establish_connection()

        self.node.create_timer(1 / self.rate, self.send_message, callback_group=MutuallyExclusiveCallbackGroup())
        self.receive_forever()

    def spin(self):
        multi_executor = MultiThreadedExecutor(num_threads=10)
        multi_executor.add_node(self.node)
        multi_executor.spin()

    def run_spin_in_thread(self):
        # Necessary in ROS2, else we are forever stuck receiving messages
        thread = threading.Thread(target=self.spin, daemon=True)
        thread.start()

    def set_state_defaults(self):
        self.gamestate: Optional[GameState] = None
        self.pose: Optional[PoseWithCovarianceStamped] = None
        self.cmd_vel: Optional[Twist] = None
        self.cmd_vel_time = Time(clock_type=self.node.get_clock().clock_type)
        self.ball: Optional[PointStamped] = None
        self.ball_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.ball_covariance: List[double] = []
        self.strategy: Optional[Strategy] = None
        self.strategy_time = Time(clock_type=self.node.get_clock().clock_type)
        self.time_to_ball: Optional[float] = None
        self.time_to_ball_time = Time(clock_type=self.node.get_clock().clock_type)
        self.seen_robots: Optional[RobotArray] = None
        self.move_base_goal: Optional[PoseStamped] = None

    def try_to_establish_connection(self):
        # we will try multiple times till we manage to get a connection
        while rclpy.ok() and not self.socket_communication.is_setup():
            self.socket_communication.establish_connection()
            self.node.get_clock().sleep_for(Duration(seconds=1))

    def create_publishers(self):
        self.team_data_publisher = self.node.create_publisher(TeamData, self.topics["team_data_topic"], qos_profile=1)

    def create_subscribers(self):
        self.node.create_subscription(
            GameState,
            self.topics["gamestate_topic"],
            self.gamestate_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            self.topics["pose_topic"],
            self.pose_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            Twist,
            self.topics["cmd_vel_topic"],
            self.cmd_vel_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            self.topics["ball_topic"],
            self.ball_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            TwistWithCovarianceStamped,
            self.topics["ball_velocity_topic"],
            self.ball_velocity_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            Strategy,
            self.topics["strategy_topic"],
            self.strategy_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            Float32,
            self.topics["time_to_ball_topic"],
            self.time_to_ball_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            RobotArray,
            self.topics["robots_topic"],
            self.robots_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.node.create_subscription(
            PoseStamped,
            self.topics["move_base_goal_topic"],
            self.move_base_goal_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def gamestate_cb(self, msg: GameState):
        self.gamestate = msg

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel = msg
        self.cmd_vel_time = self.get_current_time().to_msg()

    def strategy_cb(self, msg: Strategy):
        self.strategy = msg
        self.strategy_time = self.get_current_time().to_msg()

    def time_to_ball_cb(self, msg: Float32):
        self.time_to_ball = msg.data
        self.time_to_ball_time = self.get_current_time().to_msg()

    def move_base_goal_cb(self, msg: PoseStamped):
        self.move_base_goal = msg

    def robots_cb(self, msg: RobotArray):
        def transform_to_map(robot_relative: Robot) -> Optional[Robot]:
            # @TODO: check if this is not handled by the transform itself
            robot_pose = PoseStamped(header=msg.header, pose=robot_relative.bb.center)
            try:
                robot_on_map = self.transform_to_map_frame(robot_pose)
                robot_relative.bb.center = robot_on_map.pose
                return robot_relative
            except TransformException as err:
                self.logger.error(f"Could not transform robot to map frame: {err}")
                return None

        robots_on_map: list[Robot] = list(filter(None, map(transform_to_map, msg.robots)))
        self.seen_robots = RobotArray(header=msg.header, robots=robots_on_map)
        self.seen_robots.header.frame_id = self.map_frame

    def ball_cb(self, msg: PoseWithCovarianceStamped):
        ball_point = PointStamped(header=msg.header, point=msg.pose.pose.position)
        try:
            self.ball = self.transform_to_map_frame(ball_point)
            self.ball_covariance = msg.pose.covariance
        except TransformException as err:
            self.logger.error(f"Could not transform ball to map frame: {err}")

    def ball_velocity_cb(self, msg: TwistWithCovarianceStamped):
        self.ball_velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def transform_to_map_frame(self, field, timeout_in_s=0.3):
        return self.tf_buffer.transform(field, self.map_frame, timeout=Duration(seconds=timeout_in_s))

    def receive_forever(self):
        while rclpy.ok():
            try:
                message = self.socket_communication.receive_message()
            except (struct.error, socket.timeout):
                continue

            if message:
                self.handle_message(message)

    def handle_message(self, string_message: bytes):
        message = Proto.Message()
        message.ParseFromString(string_message)

        if self.should_message_be_discarded(message):
            self.logger.debug(
                f"Discarding msg by player {message.current_pose.player_id} "
                + f"in team {message.current_pose.team} at {message.timestamp.seconds}"
            )
            return

        team_data = self.protocol_converter.convert_from_message(message, self.create_team_data())
        self.team_data_publisher.publish(team_data)

    def send_message(self):
        if not self.is_robot_allowed_to_send_message():
            self.logger.debug("Robot is not allowed to send message")
            return

        now = self.get_current_time()
        msg = self.create_empty_message(now)

        def is_still_valid(time: Optional[TimeMsg]) -> bool:
            return (time is not None) and (now - Time.from_msg(time) < Duration(seconds=self.lifetime))

        message = self.protocol_converter.convert_to_message(self, msg, is_still_valid)
        self.socket_communication.send_message(message.SerializeToString())

    def create_empty_message(self, now: Time) -> Proto.Message:
        message = Proto.Message()
        seconds, nanoseconds = now.seconds_nanoseconds()
        message.timestamp.seconds = seconds
        message.timestamp.nanos = nanoseconds
        return message

    def create_team_data(self) -> TeamData:
        return TeamData(header=self.create_header_with_own_time())

    def create_header_with_own_time(self) -> Header:
        return Header(stamp=self.get_current_time().to_msg(), frame_id=self.map_frame)

    def should_message_be_discarded(self, message: Proto.Message) -> bool:
        player_id = message.current_pose.player_id
        team_id = message.current_pose.team

        is_own_message = player_id == self.player_id
        is_message_from_oposite_team = team_id != self.team_id

        return is_own_message or is_message_from_oposite_team

    def is_robot_allowed_to_send_message(self) -> bool:
        return self.gamestate is not None and not self.gamestate.penalized

    def get_current_time(self) -> Time:
        return self.node.get_clock().now()


def main():
    rclpy.init(args=None)
    TeamCommunication()


if __name__ == "__main__":
    main()
