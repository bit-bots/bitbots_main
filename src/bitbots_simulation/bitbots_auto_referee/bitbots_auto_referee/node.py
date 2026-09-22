"""Standalone ROS entry point composing the implemented referee components."""

import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from bitbots_auto_referee.adapters.game_controller import GameControllerUDPAdapter
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState


class AutoReferee(Node):
    def __init__(self):
        super().__init__("auto_referee")
        self.adapter: GameControllerUDPAdapter | None = None
        try:
            for name, spec in PARAMETERS.items():
                if name != "use_sim_time":
                    self.declare_parameter(
                        name, spec.default, ParameterDescriptor(description=spec.description, read_only=True)
                    )
            self.config = RefereeConfig.from_parameters({name: self.get_parameter(name).value for name in PARAMETERS})
            self.adapter = GameControllerUDPAdapter(
                target=(self.config.target_host, self.config.target_port),
                bind=(self.config.bind_host, self.config.return_port),
                state=MatchState.initial(self.config),
            )
            self._started_at = time.monotonic()
            self._connected: bool | None = None
            self._rejected_packets = 0
            self._network_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1.0 / self.config.send_rate, self._send, clock=self._network_clock)
            self.create_timer(0.05, self._receive, clock=self._network_clock)
            self.get_logger().info(
                f"AutoRef UDP target {self.config.target_host}:{self.config.target_port}; "
                f"return address {self.config.bind_host}:{self.config.return_port}. "
                f"Home {self.config.home_team_id}, away {self.config.away_team_id}; "
                f"league {self.config.league_size}, lineup {self.config.lineup_mode}, "
                f"maximum players per team {self.config.players_per_team}. "
                "Simulation observation, robot placement and automatic rule transitions are not implemented yet."
            )
        except Exception:
            self.destroy_node()
            raise

    def _send(self) -> None:
        if self.adapter is None:
            return
        try:
            self.adapter.send_state()
        except OSError as error:
            self.get_logger().error(f"GameController UDP send failed: {error}", throttle_duration_sec=5.0)

    def _receive(self) -> None:
        if self.adapter is None:
            return
        try:
            self.adapter.receive_responses()
        except OSError as error:
            self.get_logger().error(f"GameController UDP receive failed: {error}", throttle_duration_sec=5.0)

        if self.adapter.rejected_packets != self._rejected_packets:
            self.get_logger().warning(
                "Ignored invalid GameController return packets or replies outside the configured receiver/roster.",
                throttle_duration_sec=5.0,
            )
            self._rejected_packets = self.adapter.rejected_packets

        now = time.monotonic()
        last_reply = max((status.received_at for status in self.adapter.latest_responses.values()), default=None)
        connected = last_reply is not None and now - last_reply < self.config.response_timeout
        if not connected and self._connected is None and now - self._started_at < self.config.response_timeout:
            return
        if connected != self._connected:
            if connected:
                self.get_logger().info("Receiving GameController robot status replies.")
            elif last_reply is None:
                self.get_logger().info("No robot connected yet; continuing without a minimum player count.")
            else:
                self.get_logger().warning("Robot replies timed out; continuing to send the match state.")
            self._connected = connected

    def destroy_node(self):
        if self.adapter is not None:
            self.adapter.close()
            self.adapter = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoReferee()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
