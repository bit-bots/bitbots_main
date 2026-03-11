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

from construct import ConstError
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import Header
from construct import Container
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from game_controller_hsl.gamestate import GameStateStruct, ResponseStruct
from game_controller_hsl.utils import get_parameters_from_other_node
from game_controller_hsl_interfaces.msg import GameState, PlayerStatusPose


class GameStateReceiver(Node):
    """This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.

    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.

    After this we send a package back to the GC"""

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
            self.team_number = self.get_parameter("team_id").value
            self.player_number = self.get_parameter("bot_id").value
        else:
            self.get_logger().info(
                "No team_id and bot_id parameter set in game_controller, getting them from blackboard"
            )
            # Get the parameter names from the parameter server
            param_blackboard_name: str = self.get_parameter("parameter_blackboard_name").value
            team_id_param_name: str = self.get_parameter("team_id_param_name").value
            bot_id_param_name: str = self.get_parameter("bot_id_param_name").value
            # Get the parameters from the blackboard
            params = get_parameters_from_other_node(
                self, param_blackboard_name, [team_id_param_name, bot_id_param_name]
            )
            # Set the parameters
            self.team_number = params[team_id_param_name]
            self.player_number = params[bot_id_param_name]

        self.is_fallen: bool = False
        self.ball_age: float = 100 # Start with high value to show low confidence
        self.ball_position_msg: PointStamped = PointStamped()
        self.pose_msg: PlayerStatusPose = PlayerStatusPose()

        # Create subscribers
        self.create_subscription(PointStamped, "hsl_gamecontroller/ball_position", self.ball_position_cb, 1)
        self.create_subscription(Float32, "hsl_gamecontroller/ball_age", self.ball_age_cb, 1)
        self.create_subscription(Bool, "hsl_gamecontroller/is_fallen", self.is_fallen_cb, 1)
        self.create_subscription(PlayerStatusPose, "hsl_gamecontroller/pose", self.pose_cb, 1)

        self.get_logger().info(f"We are playing as player {self.player_number} in team {self.team_number}")

        # The publisher for the game state
        self.state_publisher = self.create_publisher(GameState, "gamestate", 1)

        # The publisher for the diagnostics
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "diagnostics", 1)

        # The time in seconds after which we assume the game controller is lost
        # and we tell the robot to move
        self.game_controller_lost_time = 5

        # The address listening on and the port for sending back the robots meta data
        self.addr = (self.get_parameter("listen_host").value, self.get_parameter("listen_port").value)
        self.answer_port = self.get_parameter("answer_port").value

        # The time of the last package
        self.last_package_time: Time = self.get_clock().now()

        # Create the socket we want to use for the communications
        self.socket = self._open_socket()

    def ball_position_cb(self, msg: PointStamped):
        self.ball_position_msg = msg

    def ball_age_cb(self, msg: Float32):
        self.ball_age = msg.data

    def is_fallen_cb(self, msg: Bool):
        self.is_fallen = msg.data

    def pose_cb(self, msg: PlayerStatusPose):
        self.pose_msg = msg

    def _open_socket(self) -> socket.socket:
        """Creates the socket"""
        new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        new_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        new_socket.bind(self.addr)
        new_socket.settimeout(2)
        return new_socket

    def receive_forever(self):
        """Waits in a loop for new packages"""
        while rclpy.ok():
            # Try to receive a package
            self.receive_and_answer_once()
            # Check if we didn't receive a package for a long time for publishing diagnostics
            received_message_lately = self.get_time_since_last_package() < Duration(
                seconds=self.game_controller_lost_time
            )
            self.publish_diagnostics(received_message_lately)

    def receive_and_answer_once(self):
        """Receives a package, interprets it and sends an answer."""
        try:
            # Receive the package
            data, peer = self.socket.recvfrom(GameStateStruct.sizeof())

            # Parse the package based on the GameStateStruct
            # This throws a ConstError if it doesn't work
            parsed_state = GameStateStruct.parse(data)

            # Assign the new package after it parsed successful to the state
            self.last_package_time = self.get_clock().now()

            # Build the game state message and publish it
            self.state_publisher.publish(self.build_game_state_msg(parsed_state))

            # Answer the GameController
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            self.get_logger().error(str(ae))
        except socket.timeout:
            self.get_logger().info("No GameController message received (socket timeout)", throttle_duration_sec=5)
        except ConstError:
            self.get_logger().warn("Parse Error: Probably using an old protocol!")
        except IOError as e:
            self.get_logger().warn(f"Error while sending keep-alive: {str(e)}")

    def publish_diagnostics(self, received_message_lately: bool):
        """
        This publishes a Diagnostics Array.
        """
        # initialize DiagnsticArray message
        diag_array = DiagnosticArray()

        # configure DiagnosticStatus message
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

    def answer_to_gamecontroller(self, peer):
        """Sends a life sign to the game controller"""
        # Build the answer package
        data = ResponseStruct.build(
            dict(
                player_number=self.player_number,
                team_number=self.team_number,
                fallen=self.is_fallen,
                pose=self.pose_msg.pose,
                ball_age=self.ball_age,
                ball=[self.ball_position_msg.point.x, self.ball_position_msg.point.y],
            )
        )
        # Send the package
        self.get_logger().debug(f"Sending answer to {peer[0]}:{self.answer_port}")
        try:
            self.socket.sendto(data, (peer[0], self.answer_port))
        except Exception as e:
            self.get_logger().error(f"Network Error: {str(e)}")

    def build_game_state_msg(self, state) -> GameState:
        """Builds a GameState message from the game state"""

        # Get the team objects sorted into own and rival team
        own_team = GameStateReceiver.select_team_by(lambda team: team.team_number == self.team_number, state.teams)
        rival_team = GameStateReceiver.select_team_by(lambda team: team.team_number != self.team_number, state.teams)

        # Add some assertions to make sure everything is fine
        assert not (own_team is None or rival_team is None), (
            f"Team {self.team_number} not playing, only {state.teams[0].team_number} and {state.teams[1].team_number}"
        )

        assert self.player_number <= len(own_team.players), f"Robot {self.player_number} not playing"

        this_robot = own_team.players[self.player_number - 1]

        return GameState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            players_per_team=state.players_per_team,
            competition_type=state.competition_type.intvalue,
            game_phase=state.game_phase.intvalue,
            main_state=state.state.intvalue,
            set_play=state.set_play.intvalue,
            kicking_team=state.kicking_team,
            first_half=state.first_half,
            stopped=state.stopped,
            own_score=own_team.score,
            rival_score=rival_team.score,
            secs_remaining=state.secs_remaining,
            secondary_time=state.secondary_time,
            penalized=this_robot.penalty != 0,
            seconds_till_unpenalized=this_robot.secs_till_unpenalized,
            warings=this_robot.warnings,
            cautions=this_robot.cautions,
            own_player_color=own_team.field_player_color.intvalue,
            own_goalie_color=own_team.goalkeeper_color.intvalue,
            rival_player_color=rival_team.field_player_color.intvalue,
            rival_goalie_color=rival_team.goalkeeper_color.intvalue,
            # --- Gibt es nicht mehr? ---
            # drop_in_team = state.drop_in_team,
            # drop_in_time = state.drop_in_time,
            penalty_shot=own_team.penalty_shot,
            single_shots=own_team.single_shots,
            # --- Gibt es nicht mehr? Waren die ähnlich wie Message Budget? ---
            # coach_message = own_team.coach_message,
            message_budget=own_team.message_budget,
            team_mates_with_penalty=[player.penalty != 0 for player in own_team.players],
            # --- Gibt es nicht mehr? Selber bemerken ob man rot hat? ---
            # team_mates_with_red_card = [player.number_of_red_cards != 0 for player in own_team.players],
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
