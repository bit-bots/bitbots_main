# Copyright (c) 2023 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket

import rclpy

from ros2_numpy import numpify

from construct import ConstError, Container
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from game_controller_hsl.data import GameControlDataStruct, GameControlReturnDataStruct
from game_controller_hsl.utils import get_parameters_from_other_node
from game_controller_hsl_interfaces.msg import GameState, PlayerStateResponse

from tf_transformations import euler_from_quaternion


class GameStateReceiver(Node):
    """
    This class provides a simple UDP Server which listens to packages from the game_controller on *listen_host* and *listen_port*
    and respond with the robots state on *answer_port*.

    If it receives a package it will be interpreted with the construct data structure and the :func:`build_game_state_msg`
    will be called with the content to build a ros message of type :class:`game_controller_hsl_interfaces.msg.GameState`
    which is then published on the topic */gamestate*.

    On a received ros message of type :class:`game_controller_hsl_interfaces.msg.PlayerStateResponse` on topic
    */player_state_response* the current state is updated and will be sent back to the game controller on the next
    received package.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(
            "game_controller",
            *args,
            **kwargs,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Check if we have the team and bot id parameters or if we should get them from the blackboard
        if self.has_parameter("team_id") and self.has_parameter("bot_id"):
            self.get_logger().info("Found team_id and bot_id parameter, using them")
            # Get the parameters from our node
            self.team_number: int = self.get_parameter("team_id").get_parameter_value().integer_value
            self.player_number: int = self.get_parameter("bot_id").get_parameter_value().integer_value
        else:
            self.get_logger().info(
                "No team_id and bot_id parameter set in game_controller, getting them from blackboard"
            )
            # Get the parameter names from the parameter server
            param_blackboard_name: str = self.get_parameter("parameter_blackboard_name").get_parameter_value().string_value
            team_id_param_name: str = self.get_parameter("team_id_param_name").get_parameter_value().string_value
            bot_id_param_name: str = self.get_parameter("bot_id_param_name").get_parameter_value().string_value

            params = get_parameters_from_other_node(
                self, param_blackboard_name, [team_id_param_name, bot_id_param_name]
            )
            self.team_number = params[team_id_param_name]
            self.player_number = params[bot_id_param_name]

        self.player_state_msg: PlayerStateResponse = PlayerStateResponse()
        self.create_subscription(PlayerStateResponse, "player_state_response", self.player_state_cb, 1)

        self.get_logger().info(f"We are playing as player {self.player_number} in team {self.team_number}")

        self.state_publisher = self.create_publisher(GameState, "gamestate", 1)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "diagnostics", 1)

        # The time in seconds after which we assume the game controller is lost and publish a warning in the diagnostics
        self.game_controller_lost_timeout: int = self.get_parameter("lost_timeout").get_parameter_value().integer_value

        # The address listening on and the port for sending back the robots meta data
        self.listening_address = (self.get_parameter("listen_host").value, self.get_parameter("listen_port").value)
        self.answer_port = self.get_parameter("answer_port").value

        # The time of the last package
        self.last_package_time: Time = self.get_clock().now()

        # Create the socket we want to use for the communications
        self.socket = self._open_socket(self.listening_address)

    def player_state_cb(self, msg: PlayerStateResponse):
        self.player_state_msg = msg

    def _open_socket(self, address: tuple[str, str]) -> socket.socket:
        """Creates a UDP socket to listen on the given address and port with a timeout of 2 seconds."""
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.bind(address)
        udp_socket.settimeout(2)

        return udp_socket

    def receive_forever(self):
        """Waits in a loop for new packages from the game controller."""
        while rclpy.ok():
            # Try to receive a package
            self.receive_and_answer_once()
            # Check if we didn't receive a package for a long time for publishing diagnostics
            received_message_lately = self.get_time_since_last_package() < Duration(
                seconds=self.game_controller_lost_timeout
            )
            self.publish_diagnostics(received_message_lately)
            rclpy.spin_once(self, timeout_sec=0.1)

    def receive_and_answer_once(self):
        """
        Receives a package, parses it and publishes the GameState.
        Then sends back the current player state to the game controller.
        """
        try:
            # Receive the package
            data, peer = self.socket.recvfrom(GameControlDataStruct.sizeof())

            # Parse the package based on the game controllers GameControlDataStruct.
            # This throws a ConstError if it doesn't work
            parsed_state = GameControlDataStruct.parse(data)
            self.last_package_time = self.get_clock().now()

            self.state_publisher.publish(self.build_game_state_msg(parsed_state))
            self.answer_to_game_controller(peer)

        except AssertionError as e:
            self.get_logger().error(str(e))
        except TimeoutError:
            self.get_logger().info("No GameController message received (socket timeout)", throttle_duration_sec=5)
        except ConstError as e:
            self.get_logger().error(f"Parse Error: Probably using an old protocol! {str(e)}")
        except OSError as e:
            self.get_logger().warn(f"Error while sending keep-alive: {str(e)}")

    def publish_diagnostics(self, received_message_lately: bool):
        """
        This publishes a Diagnostics Array message with the status of the game controller connection.
        """
        diag_array = DiagnosticArray()
        diag = DiagnosticStatus(name="Game Controller", hardware_id="Game Controller")

        if not received_message_lately:
            self.get_logger().info("No GameController message received", throttle_duration_sec=5)
            diag.message = (
                "Lost connection to game controller for "
                + str(int(self.get_time_since_last_package().nanoseconds / 1e9))
                + " sec"
            )
            diag.level = DiagnosticStatus.WARN
        else:
            diag.message = "Connected"
            diag.level = DiagnosticStatus.OK

        diag_array.status.append(diag)

        # add timestamp to header and publish DiagnosticArray
        diag_array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostic_pub.publish(diag_array)

    def build_game_control_return_data(self, player_state: PlayerStateResponse) -> bytes:
        """
        Convert a player state message into a serialized GameControlReturnData packet.
        Which expects standardized units of millimeters and uses the euler yaw angle for player orientation.
        """
        pose = [
            player_state.pose.pose.position.x * 1000.0,
            player_state.pose.pose.position.y * 1000.0,
            euler_from_quaternion(numpify(player_state.pose.pose.orientation))[2]
        ]

        ball_age = -1.0
        ball_timestamp = Time.from_msg(player_state.ball.header.stamp)
        if ball_timestamp.nanoseconds > 0:
            ball_age = (self.get_clock().now() - ball_timestamp).nanoseconds / 1e9

        ball = [
            player_state.ball.point.x * 1000.0,
            player_state.ball.point.y * 1000.0,
        ]

        return GameControlReturnDataStruct.build(
            dict(
                player_number=self.player_number,
                team_number=self.team_number,
                fallen=player_state.fallen,
                pose=pose,
                ball_age=ball_age,
                ball=ball
            )
        )

    def answer_to_game_controller(self, peer):
        """Send the latest player state to the GameController."""
        data = self.build_game_control_return_data(self.player_state_msg)

        self.get_logger().debug(f"Sending answer to {peer[0]}:{self.answer_port}")
        try:
            self.socket.sendto(data, (peer[0], self.answer_port))
        except Exception as e:
            self.get_logger().error(f"Network Error: {str(e)}")

    def build_game_state_msg(self, game_control_data) -> GameState:
        """Builds a GameState ros message from the game state received from the game controller"""

        # Get the team objects sorted into own and rival team
        own_team = GameStateReceiver.select_team_by(lambda team: team.team_number == self.team_number, game_control_data.teams)
        rival_team = GameStateReceiver.select_team_by(lambda team: team.team_number != self.team_number, game_control_data.teams)

        # Add some assertions to make sure everything is fine
        assert not (own_team is None or rival_team is None), (
            f"Team {self.team_number} not playing, only {game_control_data.teams[0].team_number} and {game_control_data.teams[1].team_number}"
        )

        assert self.player_number <= len(own_team.players), f"Robot {self.player_number} not playing"

        this_robot = own_team.players[self.player_number - 1]

        return GameState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            players_per_team=game_control_data.players_per_team,
            competition_type=game_control_data.competition_type.intvalue,
            game_phase=game_control_data.game_phase.intvalue,
            main_state=game_control_data.state.intvalue,
            set_play=game_control_data.set_play.intvalue,
            kicking_team=game_control_data.kicking_team,
            first_half=game_control_data.first_half,
            stopped=game_control_data.stopped,
            own_score=own_team.score,
            rival_score=rival_team.score,
            seconds_remaining=game_control_data.secs_remaining,
            secondary_time=game_control_data.secondary_time,
            penalized=this_robot.penalty.intvalue != GameState.PENALTY_NONE,
            # In the current rules the only in place penalty is the motion in set
            penalized_in_place=this_robot.penalty.intvalue == GameState.PENALTY_MOTION_IN_SET,
            seconds_till_unpenalized=this_robot.secs_till_unpenalized,
            cautions=this_robot.cautions,
            has_yellow_card=this_robot.cautions== 1,
            has_red_card=this_robot.penalty.intvalue == GameState.PENALTY_SENT_OFF,
            own_player_color=own_team.field_player_color.intvalue,
            own_goalie_color=own_team.goalkeeper_color.intvalue,
            rival_player_color=rival_team.field_player_color.intvalue,
            rival_goalie_color=rival_team.goalkeeper_color.intvalue,
            penalty_shot=own_team.penalty_shot,
            single_shots=own_team.single_shots,
            message_budget=own_team.message_budget,
            team_mates_with_penalty=[player.penalty.intvalue != GameState.PENALTY_NONE for player in own_team.players],
            team_mates_with_yellow_card = [player.cautions == 1 for player in own_team.players],
            team_mates_with_red_card = [player.penalty.intvalue == GameState.PENALTY_SENT_OFF for player in own_team.players],
        )


    def get_time_since_last_package(self) -> Duration:
        """Returns the time in seconds since the last package was received"""
        return self.get_clock().now() - self.last_package_time

    @staticmethod
    def select_team_by(predicate: callable, teams: list[Container]) -> Container:
        """Selects the team based on the predicate"""
        selected = [team for team in teams if predicate(team)]
        return next(iter(selected), None)


def main(args=None):
    rclpy.init(args=args)
    receiver = GameStateReceiver()

    try:
        receiver.receive_forever()
    except KeyboardInterrupt:
        receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
