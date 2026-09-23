"""Standalone ROS entry point composing the implemented referee components."""

import time

import rclpy
from bitbots_msgs.msg import SimulationState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from bitbots_auto_referee.adapters.game_controller import GameControllerUDPAdapter
from bitbots_auto_referee.adapters.simulation.commands import SimulationCommands
from bitbots_auto_referee.adapters.simulation.observations import decode_observation
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.check_rules import RuleChecker
from bitbots_auto_referee.rules.outside import FieldGeometry
from bitbots_auto_referee.rules.startup import StartupSequence
from bitbots_auto_referee.ui.dashboard import Dashboard


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
            )
            self._started_at = time.monotonic()
            self._connected: bool | None = None
            self._rejected_packets = 0
            self._network_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._startup = StartupSequence()
            self.simulation_commands = SimulationCommands(self, self._record_event)
            self.rule_checker = RuleChecker(
                self.config.robot_teams,
                event_callback=self._record_event,
                teleport_commands=self.simulation_commands,
                field=FieldGeometry.from_config(self.config),
            )
            self.dashboard = Dashboard(self)
            self._record_event("AutoRef gestartet: INITIAL")
            self._refresh_dashboard()
            self.create_timer(0.25, self._refresh_dashboard, clock=self._network_clock)
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
                "Robot and ball positions and contacts are observed; robot placement is not implemented yet."
            )
        except Exception:
            self.destroy_node()
            raise

    def _on_simulation_step(self, message: SimulationState) -> None:
        """Evaluate rules in every phase whenever the simulator completes a step."""
        if self.adapter is None:
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
        if self.adapter is None or self._startup.finished:
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
            )

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
                self._record_event("Roboterantworten werden empfangen.")
            elif last_reply is None:
                self._record_event("Noch kein Roboter verbunden; AutoRef läuft weiter.")
            else:
                self.get_logger().warning("Robot replies timed out; continuing to send the match state.")
                self._record_event("Roboterverbindung unterbrochen; AutoRef läuft weiter.")
            self._connected = connected

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
