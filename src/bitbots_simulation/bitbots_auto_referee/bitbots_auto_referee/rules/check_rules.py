"""Entry point for rule evaluation after a simulation update."""

from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import replace

from bitbots_auto_referee.core.observations import Position, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportCommands, TeleportResult


class RuleChecker:
    """Keep current ground truth and recognize the beginning of ball contacts."""

    def __init__(
        self,
        robot_teams: dict[int, int],
        event_callback: Callable[[str], None] | None = None,
        teleport_commands: TeleportCommands | None = None,
    ):
        self._teleport_commands = teleport_commands
        self._event_callback = event_callback
        self.robot_teams = dict(robot_teams)
        self.robot_positions: dict[int, Position] = {}
        self.ball_position: Position | None = None
        self.simulation_time_ns: int | None = None
        self.step_number: int | None = None
        self.last_touch_team_id: int | None = None
        self.unmapped_robot_indices: set[int] = set()
        self._touching_ball: frozenset[int] = frozenset()
        self._clock_was_running = False
        self._fractional_time_ns = 0
        self._expected_seconds: int | None = None
        self._last_check_ball_outside: bool = False 

    def _update_clock(self, game_state: MatchState, observation: SimulationObservation) -> MatchState:
        """Accumulate only active simulation intervals, retaining fractions across stops."""
        running = game_state.state == "STATE_PLAYING" and not game_state.stopped
        reset = self.simulation_time_ns is None or (
            observation.time_ns < self.simulation_time_ns
            or (self.step_number is not None and observation.step_number < self.step_number)
        )
        if reset or game_state.secs_remaining != self._expected_seconds:
            self._fractional_time_ns = 0
        elif running and self._clock_was_running:
            self._fractional_time_ns += observation.time_ns - self.simulation_time_ns
        self._clock_was_running = running
        elapsed_seconds, self._fractional_time_ns = divmod(self._fractional_time_ns, 1_000_000_000)
        remaining = max(0, game_state.secs_remaining - elapsed_seconds)
        if remaining == 0:
            self._fractional_time_ns = 0
        self._expected_seconds = remaining
        return replace(game_state, secs_remaining=remaining) if remaining != game_state.secs_remaining else game_state

    def check_rules(self, game_state: MatchState, observation: SimulationObservation) -> MatchState:
        """Update the playing clock and positions before handling new ball contacts."""
        previous_seconds = game_state.secs_remaining
        game_state = self._update_clock(game_state, observation)
        if (
            self.step_number is not None
            and self.simulation_time_ns is not None
            and (observation.step_number < self.step_number or observation.time_ns < self.simulation_time_ns)
        ):
            self._touching_ball = frozenset()
            self.last_touch_team_id = None
        self.robot_positions = dict(observation.robot_positions)
        self.ball_position = observation.ball_position
        self.simulation_time_ns = observation.time_ns
        self.step_number = observation.step_number
        if previous_seconds > 0 and game_state.secs_remaining == 0 and self._event_callback is not None:
            self._event_callback("Spielzeit abgelaufen. Ein automatischer Halbzeitwechsel ist noch nicht implementiert.")
        self.unmapped_robot_indices = self.robot_positions.keys() - self.robot_teams.keys()
        touching = (
            observation.touching_ball.intersection(self.robot_positions)
            if observation.ball_position is not None
            else frozenset()
        )
        new_contacts = touching - self._touching_ball - observation.teleported_robots
        if observation.ball_teleported:
            new_contacts = frozenset()
            self.last_touch_team_id = None
        self._touching_ball = touching
        for robot_index in sorted(new_contacts):
            team_id = self.robot_teams.get(robot_index)
            if team_id is not None:
                self.on_robot_ball_contact(team_id)

        """Checking game rules"""
        if(not self._last_check_ball_outside and self.ballOutside()):
            game_state = self.handleBalloutside(game_state)
            self._last_check_ball_outside = True

        return game_state

    def ballOutside(self) -> bool:
        #prüft ob ball außerhalb der feld linien gemäß config
        return False

    def handleBalloutside(self, game_state: MatchState) -> MatchState:
        #prüft wo der ball ins ausgegangen ist (Tor, Torlinie, Seitenaus)
        # gibt je anch letzter berührung Tor, Einwurf, Ecke oder goal_kick (abstoß), lässt aber zunächst noch 2 sekunden das Spiel laufen
        # bei Tor wird im gamesate das Tor bei den teams ergenzt und die sequenz von ready set playing startet ernert, dabei geht der anstoß an die Team ID die das tor kassiert hat
        # bei einwurf ecke oder goal kick wechselt der set_play zur entsprechenden ID (siehe gamecontroler), die secondary_time wird auf 45 sekunden gestellt
        # replatziert den Ball entweder auf dem anstoßpunkt (Tor), an der Ecke, an der fünf Meter raum ecke (goal kick), am nähesten Punkt der seitenauslinie (einwurf)
        return game_state


    def teleport_robot(self, robot_id: int, x: float, y: float, yaw: float) -> Future[TeleportResult]:
        """Request an absolute planar teleport by simulator index; never wait inside check_rules."""
        if self._teleport_commands is None:
            raise RuntimeError("No simulator command adapter configured")
        return self._teleport_commands.teleport_robot(robot_id, x, y, yaw)

    def teleport_ball(self, x: float, y: float, yaw: float) -> Future[TeleportResult]:
        """Request a ball teleport; yaw changes orientation even for a spherical ball."""
        if self._teleport_commands is None:
            raise RuntimeError("No simulator command adapter configured")
        return self._teleport_commands.teleport_ball(x, y, yaw)

    def on_robot_ball_contact(self, team_id: int) -> None:
        """Handle a new robot-ball contact; repeated contact points count as one touch."""
        self.last_touch_team_id = team_id
        if self._event_callback is not None:
            self._event_callback(f"Ballkontakt: Team {team_id}")
