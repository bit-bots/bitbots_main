"""Standalone ROS entry point composing the implemented referee components."""

import json
import time
from functools import partial

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from std_msgs.msg import String

from bitbots_auto_referee.adapters.game_controller import GameControllerUDPAdapter
from bitbots_auto_referee.adapters.simulation.commands import SimulationCommands
from bitbots_auto_referee.adapters.simulation.observations import decode_observation
from bitbots_auto_referee.adapters.simulation.roster import validate_roster
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.communication import MessageBudget, robot_connections
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.check_rules import RuleChecker
from bitbots_auto_referee.rules.double_touch import DoubleTouchConfig
from bitbots_auto_referee.rules.outside import FieldGeometry
from bitbots_auto_referee.rules.pushing import PushingConfig
from bitbots_auto_referee.rules.startup import StartupSequence
from bitbots_auto_referee.ui.dashboard import Dashboard, dashboard_qos
from bitbots_msgs.msg import SimulationState


class AutoReferee(Node):
    def __init__(self):
        super().__init__("auto_referee")
        self.adapter: GameControllerUDPAdapter | None = None
        self.dashboard: Dashboard | None = None
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
                broadcast=self.config.broadcast,
            )
            self._started_at = time.monotonic()
            self._connected: bool | None = None
            self._robot_connections: dict[tuple[int, int], bool | None] = {}
            self._budget = MessageBudget()
            self._roster: dict[int, dict] = {}
            self._send_subscriptions: dict[int, Subscription] = {}
            self._roster_valid = not (self.config.num_robots or self.get_parameter("use_sim_time").value)
            self._roster_status = "Standalone" if self._roster_valid else "Warte auf Simulator-Aufstellung"
            self._rejected_packets = 0
            self._network_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._startup = StartupSequence()
            self.simulation_commands = SimulationCommands(self, self._record_event)
            self.rule_checker = RuleChecker(
                self.config.robot_teams,
                event_callback=self._record_event,
                teleport_commands=self.simulation_commands,
                field=FieldGeometry.from_config(self.config),
                robot_players=self.config.robot_players,
                pushing_config=PushingConfig.from_config(self.config),
                double_touch_config=DoubleTouchConfig.from_config(self.config),
            )
            self.dashboard = Dashboard(self)
            self._record_event("AutoRef gestartet: INITIAL")
            self._refresh_dashboard()
            self.create_timer(0.25, self._refresh_dashboard, clock=self._network_clock)
            self.create_subscription(String, "/simulation/roster", self._on_roster, dashboard_qos())
            self._unmapped_robot_indices: set[int] = set()
            self.create_subscription(
                SimulationState,
                "/simulation/step",
                self._on_simulation_step,
                QoSProfile(
                    history=HistoryPolicy.KEEP_ALL,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self._advance_startup()
            self.create_timer(0.05, self._advance_startup, clock=self._network_clock)
            self.create_timer(1.0 / self.config.send_rate, self._send, clock=self._network_clock)
            self.create_timer(0.05, self._receive, clock=self._network_clock)
            self.get_logger().info(
                f"AutoRef UDP target {self.config.target_host}:{self.config.target_port}; "
                f"return address {self.config.bind_host}:{self.config.return_port}. "
                f"Home {self.config.home_team_id}, away {self.config.away_team_id}; "
                f"league {self.config.league_size}, lineup {self.config.lineup_mode}, "
                f"maximum players per team {self.config.players_per_team}. "
                "The opening sequence advances automatically to PLAYING. "
                "Robot and ball positions and contacts are observed."
            )
        except Exception:
            self.destroy_node()
            raise

    def _on_simulation_step(self, message: SimulationState) -> None:
        """Evaluate rules in every phase whenever the simulator completes a step."""
        if self.adapter is None or not self._roster_valid:
            return
        previous = self.adapter.state
        updated = self.rule_checker.check_rules(previous, decode_observation(message))
        unmapped = self.rule_checker.unmapped_robot_indices
        if unmapped != self._unmapped_robot_indices:
            if unmapped:
                self.get_logger().warning(
                    f"No team mapping for robot indices {sorted(unmapped)}; their ball-contact callbacks are skipped."
                )
            self._unmapped_robot_indices = set(unmapped)
        if updated != previous:
            self.adapter.set_state(updated)
            if updated.state != previous.state or updated.stopped != previous.stopped:
                self._record_event(f"Regelentscheidung: {updated.state}, stopped={updated.stopped}")
            self._send()

    def _advance_startup(self) -> None:
        if self.adapter is None or not self._roster_valid or self._startup.finished:
            return
        if self.get_parameter("use_sim_time").value:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns == 0:
                return
        else:
            now_ns = self._network_clock.now().nanoseconds
        previous = self.adapter.state
        updated = self._startup.advance(previous, now_ns)
        if updated != previous:
            self.adapter.set_state(updated)
            if updated.state != previous.state:
                self._record_event(f"Startablauf: {previous.state} → {updated.state}")
                self._send()

    def _record_event(self, message: str) -> None:
        self.get_logger().info(message)
        if self.dashboard is not None:
            self.dashboard.record(message, self.rule_checker.simulation_time_ns)

    def _refresh_dashboard(self) -> None:
        if self.dashboard is not None and self.adapter is not None:
            self.dashboard.update(
                self.adapter.state,
                self.rule_checker.simulation_time_ns,
                self.rule_checker.last_touch_team_id,
                self._connected,
                self._robot_connections,
                self._roster_status,
            )

    def _on_roster(self, message: String) -> None:
        try:
            roster = validate_roster(self.config, json.loads(message.data))
        except (ValueError, TypeError, KeyError) as error:
            self._roster_valid = False
            status = f"Aufstellung ungültig: {error}"
            if status != self._roster_status:
                self._roster_status = status
                self.get_logger().error(status)
                self._record_event(status)
            return
        if roster == self._roster and self._roster_valid:
            return
        for subscription in self._send_subscriptions.values():
            self.destroy_subscription(subscription)
        self._send_subscriptions.clear()
        self._roster = roster
        self._roster_valid = True
        self._roster_status = "Simulator-Aufstellung geprüft"
        for index, robot in roster.items():
            self._send_subscriptions[index] = self.create_subscription(
                String,
                f"/robot{robot['domain']}/team_communication/sent",
                partial(self._on_team_message_count, index),
                dashboard_qos(),
            )
        self._record_event(self._roster_status)
        self._send()

    def _on_team_message_count(self, index: int, message: String) -> None:
        if self.adapter is None or not self._roster_valid or index not in self._roster:
            return
        try:
            report = json.loads(message.data)
            robot = self._roster[index]
            if (report["team_id"], report["player_id"]) != (robot["team_id"], robot["bot_id"]):
                raise ValueError("Sender identity does not match the simulator roster")
            previous = self.adapter.state
            updated = self._budget.update(
                previous, robot["team_id"], robot["bot_id"], report["session"], report["sent"]
            )
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().error(f"Invalid team send report: {error}", throttle_duration_sec=5.0)
            return
        if updated != previous:
            self.adapter.set_state(updated)
            self._send()

    def _send(self) -> None:
        if self.adapter is None or not self._roster_valid:
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
        expected = {(team, self.config.robot_players[index]) for index, team in self.config.robot_teams.items()}
        connections = robot_connections(
            expected, self.adapter.latest_responses, now, self._started_at, self.config.response_timeout
        )
        for (team, player), connected in connections.items():
            if connected is not None and connected != self._robot_connections.get((team, player)):
                status = "verbunden" if connected else "keine aktuelle Antwort"
                event = f"Team {team}, Spieler {player}: {status}"
                self._record_event(event)
                if not connected:
                    self.get_logger().warning(event)
        self._robot_connections = connections
        if False in connections.values():
            self._connected = False
        elif None in connections.values() or not connections:
            self._connected = None
        else:
            self._connected = True

    def destroy_node(self):
        if self.dashboard is not None:
            self.dashboard.close()
            self.dashboard = None
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
