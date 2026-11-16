"""Standalone ROS entry point composing the implemented referee components."""

import json
import time
from dataclasses import replace

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bitbots_auto_referee.adapters.game_controller import GameControllerUDPAdapter
from bitbots_auto_referee.adapters.simulation.commands import SimulationCommands
from bitbots_auto_referee.adapters.simulation.observations import decode_observation
from bitbots_auto_referee.adapters.simulation.team_messages import TeamMessageReceiver
from bitbots_auto_referee.config import LIVE_PARAMETERS, PARAMETERS, RefereeConfig, field_center_circle_radius
from bitbots_auto_referee.core.communication import count_team_messages, robot_connections
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
        self._team_messages: TeamMessageReceiver | None = None
        try:
            for name, spec in PARAMETERS.items():
                if name != "use_sim_time":
                    default = spec.default
                    if name == "center_circle_radius" and self.get_parameter("field_config").value:
                        default = field_center_circle_radius(self.get_parameter("field_config").value)
                    self.declare_parameter(
                        name,
                        default,
                        ParameterDescriptor(description=spec.description, read_only=name not in LIVE_PARAMETERS),
                        ignore_override=name == "center_circle_radius" and bool(self.get_parameter("field_config").value),
                    )
            self.config = RefereeConfig.from_parameters({name: self.get_parameter(name).value for name in PARAMETERS})
            self.set_parameters([Parameter("set_play_min_distance", value=self.config.set_play_min_distance)])
            self.adapter = GameControllerUDPAdapter(
                target=(self.config.target_host, self.config.target_port),
                bind=(self.config.bind_host, self.config.return_port),
                state=MatchState.initial(self.config),
                broadcast=self.config.broadcast,
            )
            self._started_at = time.monotonic()
            self._connected: bool | None = None
            self._robot_connections: dict[tuple[int, int], bool | None] = {}
            self._roster: dict[int, dict] = {}
            self._roster_status = (
                "Warte auf Simulator-Aufstellung"
                if self.config.num_robots or self.get_parameter("use_sim_time").value
                else "Standalone"
            )
            self._rejected_packets = 0
            self._network_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._team_messages = TeamMessageReceiver((self.config.home_team_id, self.config.away_team_id))
            self.create_timer(0.01, self._receive_team_messages, clock=self._network_clock)
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
                set_play_min_distance=self.config.set_play_min_distance,
            )
            self.add_on_set_parameters_callback(self._validate_live_parameters)
            self.add_post_set_parameters_callback(self._apply_live_parameters)
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

    def _validate_live_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "set_play_min_distance" and (
                type(parameter.value) is not float or not parameter.value > 0
            ):
                return SetParametersResult(successful=False, reason="Set-play distance must be positive")
        values = {name: self.get_parameter(name).value for name in PARAMETERS}
        values.update({parameter.name: parameter.value for parameter in parameters})
        try:
            RefereeConfig.from_parameters(values)
        except (ValueError, TypeError, KeyError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _apply_live_parameters(self, parameters) -> None:
        changes = {parameter.name: parameter.value for parameter in parameters if parameter.name in LIVE_PARAMETERS}
        if not changes or all(getattr(self.config, name) == value for name, value in changes.items()):
            return
        changes = {name: value for name, value in changes.items() if getattr(self.config, name) != value}
        self.config = replace(self.config, **changes)
        if any(name.startswith("pushing_") for name in changes):
            self.rule_checker.pushing_rules.configure(PushingConfig.from_config(self.config))
            self._record_event(
                "Pushing-Schwellwerte geändert: "
                f"anhaltender Kontakt {self.config.pushing_sustained_force_threshold:g} N; "
                f"Einzelkontakt {self.config.pushing_force_threshold:g} N."
            )
        if "set_play_min_distance" in changes:
            self.rule_checker.robot_position_rules.configure_set_play_distance(self.config.set_play_min_distance)
            self._record_event(f"Set-Play-Mindestabstand geändert: {self.config.set_play_min_distance:g} m.")
        self._refresh_dashboard()

    def _on_simulation_step(self, message: SimulationState) -> None:
        """Evaluate rules in every phase whenever the simulator completes a step."""
        if self.adapter is None:
            return
        previous = self.adapter.state
        updated = self.rule_checker.check_rules(previous, decode_observation(message))
        if previous.state == "STATE_READY" and updated.state == "STATE_SET" and not self._startup.finished:
            clock = self.get_clock() if self.get_parameter("use_sim_time").value else self._network_clock
            self._startup.finish_ready(clock.now().nanoseconds)
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
        if self.adapter is None or self._startup.finished:
            return
        if self.get_parameter("use_sim_time").value:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns == 0:
                return
        else:
            now_ns = self._network_clock.now().nanoseconds
        previous = self.adapter.state
        if previous.state in ("STATE_INITIAL", "STATE_READY"):
            self.rule_checker.kickoff_ball.arm()
        updated = self._startup.advance(
            previous,
            now_ns,
            paused=bool(self.rule_checker.robot_placements.pending)
            or (previous.state == "STATE_SET" and self.rule_checker.kickoff_ball.waiting),
            ready_for_play=not self.rule_checker.kickoff_ball.waiting,
        )
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
                {name: getattr(self.config, name) for name in LIVE_PARAMETERS},
            )

    def _on_roster(self, message: String) -> None:
        roster = {robot["index"]: robot for robot in json.loads(message.data)}
        if roster == self._roster:
            return
        self._roster = roster
        self._roster_status = "Simulator-Aufstellung übernommen"
        self._record_event(self._roster_status)

    def _receive_team_messages(self) -> None:
        if self.adapter is None or self._team_messages is None:
            return
        previous = self.adapter.state
        updated = count_team_messages(previous, self._team_messages.receive_counts())
        if updated != previous:
            self.adapter.set_state(updated)
            self._send()

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
        if self._team_messages is not None:
            self._team_messages.close()
            self._team_messages = None
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
