"""Entry point for rule evaluation after a simulation update."""

import math
from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import replace

from bitbots_auto_referee.core.observations import Position, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportCommands, TeleportResult
from bitbots_auto_referee.rules.double_touch import DoubleTouchConfig, DoubleTouchRules
from bitbots_auto_referee.rules.game_flow import GameFlowRules
from bitbots_auto_referee.rules.motion import MotionRules
from bitbots_auto_referee.rules.outside import OUTSIDE_DELAY_NS, SET_PLAY_SECONDS, FieldGeometry, OutsideDecision
from bitbots_auto_referee.rules.pushing import PushingConfig, PushingRules
from bitbots_auto_referee.rules.robot_position import RobotPositionRules
from bitbots_auto_referee.rules.startup import NANOSECONDS_PER_SECOND, STARTUP_PHASES


class RuleChecker:
    """Keep current ground truth and recognize the beginning of ball contacts."""

    def __init__(
        self,
        robot_teams: dict[int, int],
        event_callback: Callable[[str], None] | None = None,
        teleport_commands: TeleportCommands | None = None,
        field: FieldGeometry | None = None,
        robot_players: dict[int, int] | None = None,
        pushing_config: PushingConfig | None = None,
        double_touch_config: DoubleTouchConfig | None = None,
    ):
        self._teleport_commands = teleport_commands
        self._event_callback = event_callback
        self.robot_teams = dict(robot_teams)
        self.robot_positions: dict[int, Position] = {}
        self.ball_position: Position | None = None
        self.simulation_time_ns: int | None = None
        self.step_number: int | None = None
        self.last_touch_team_id: int | None = None
        self._indirect_team: int | None = None
        self._indirect_first_robot: int | None = None
        self.unmapped_robot_indices: set[int] = set()
        self._touching_ball: frozenset[int] = frozenset()
        self._clock_was_running = False
        self._fractional_time_ns = 0
        self._expected_seconds: int | None = None
        self.field = field or FieldGeometry()
        self._last_check_ball_outside = False
        self._previous_ball_position: Position | None = None
        self._pending_outside: OutsideDecision | None = None
        self._placement: Future[TeleportResult] | None = None
        self._restart_at: int | None = None
        self._restart_is_goal = False
        if robot_players is None:
            robot_players = {}
            for team in set(self.robot_teams.values()):
                indices = sorted(index for index, team_id in self.robot_teams.items() if team_id == team)
                robot_players.update({index: number for number, index in enumerate(indices, 1)})
        self.double_touch_rules = DoubleTouchRules(
            self.robot_teams, robot_players, self.field, self._event, double_touch_config
        )
        self.motion_rules = MotionRules(self.robot_teams, robot_players, self.field, self.teleport_robot, self._event)
        self.robot_position_rules = RobotPositionRules(self.robot_teams, robot_players, self.field, self.motion_rules)
        self._placement_failed = False
        self.pushing_rules = PushingRules(self.motion_rules, self._event, pushing_config)
        self.game_flow_rules = GameFlowRules(
            self.motion_rules, self.field, self.teleport_robot, self.teleport_ball, self._event
        )

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
        if game_state.state != "STATE_PLAYING":
            self._clear_indirect()
        previous_seconds = game_state.secs_remaining
        game_state = self._update_clock(game_state, observation)
        if (
            self.step_number is not None
            and self.simulation_time_ns is not None
            and (observation.step_number < self.step_number or observation.time_ns < self.simulation_time_ns)
        ):
            self._touching_ball = frozenset()
            self.last_touch_team_id = None
            self._reset_outside()
        self._previous_ball_position = self.ball_position
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

        contact_teams = {self.robot_teams.get(index) for index in new_contacts}
        if None in contact_teams or len(contact_teams) > 1:
            self.last_touch_team_id = None
        if observation.ball_teleported:
            if self._placement is None and not self._placement_failed:
                self._pending_outside = None
                self._clear_indirect()
                self.double_touch_rules.clear()
            self._previous_ball_position = None
            self._last_check_ball_outside = self.ballOutside()
        if game_state.state == "STATE_PLAYING" and not game_state.stopped and self._pending_outside is None:
            self._track_indirect_contacts(new_contacts)
        double_touch_enabled = (
            game_state.state == "STATE_PLAYING" and not game_state.stopped
            and self._pending_outside is None and self._placement is None
        )
        # Only contacts observed after a previously acknowledged placement can end a restart.
        restart_active = self._restart_at is not None and not self._restart_is_goal and self._pending_outside is None
        game_state = self._check_outside(game_state)
        if (
            restart_active
            and self._restart_at is not None
            and self._pending_outside is None
            and game_state.state == "STATE_PLAYING"
            and not game_state.stopped
            and game_state.secondary_time > 0
            and game_state.set_play != "SET_PLAY_NONE"
            and game_state.kicking_team in contact_teams
        ):
            self._restart_at = None
            game_state = replace(game_state, set_play="SET_PLAY_NONE", secondary_time=0)
            self._event(f"Standardsituation ausgeführt: Ballkontakt durch Team {game_state.kicking_team}")
        game_state = self.motion_rules.check(game_state, observation)
        game_state = self.robot_position_rules.check(game_state, observation)
        fouling_team = self.double_touch_rules.check(game_state, observation, enabled=double_touch_enabled)
        if fouling_team is not None:
            opponent = next(team.team_number for team in game_state.teams if team.team_number != fouling_team)
            game_state = self.award_indirect_free_kick(game_state, opponent, *observation.ball_position[:2])
        before_flow = game_state
        game_state = self.game_flow_rules.check(game_state, observation)
        if game_state.kicking_team == 255 and game_state != before_flow:
            self._clear_indirect()
            self.double_touch_rules.clear()
        return self.pushing_check(game_state, observation)

    def pushing_check(self, game_state: MatchState, observation: SimulationObservation) -> MatchState:
        game_state = self.pushing_rules.check(game_state, observation)
        if game_state.state == "STATE_PLAYING" and not game_state.stopped:
            for foul in self.pushing_rules.fouls:
                team_id = self.robot_teams.get(foul.victim)
                if team_id is not None:
                    return self.award_direct_free_kick(game_state, team_id, *foul.position)
        return game_state

    def award_direct_free_kick(self, game_state: MatchState, team_id: int, x: float, y: float) -> MatchState:
        """Award a direct free kick at the incident location during active play."""
        return self._award_free_kick(game_state, team_id, x, y, "SET_PLAY_DIRECT_FREE_KICK")

    def award_indirect_free_kick(self, game_state: MatchState, team_id: int, x: float, y: float) -> MatchState:
        """Award the opponent an indirect restart, including for a detected double touch."""
        return self._award_free_kick(game_state, team_id, x, y, "SET_PLAY_INDIRECT_FREE_KICK")

    def _award_free_kick(self, game_state, team_id, x, y, kind):
        if team_id not in {team.team_number for team in game_state.teams}:
            raise ValueError("Free kick requires a configured team")
        if not all(math.isfinite(value) for value in (x, y)):
            raise ValueError("Free-kick position must be finite")
        if self.simulation_time_ns is None:
            raise RuntimeError("A simulation observation is required before awarding a free kick")
        if game_state.state != "STATE_PLAYING" or game_state.stopped:
            raise ValueError("Free kicks can only be awarded during active PLAYING")
        if self._placement is not None or self._placement_failed:
            raise RuntimeError("Cannot replace an outstanding or failed ball placement")
        self._restart_at = None
        self._clear_indirect()
        self.double_touch_rules.clear()
        # Foul restarts start immediately; the outside-ball decision delay does not apply.
        self._pending_outside = OutsideDecision(
            kind, team_id, (float(x), float(y)), self.simulation_time_ns - OUTSIDE_DELAY_NS
        )
        return self.handleBalloutside(game_state)

    def _clear_indirect(self) -> None:
        self._indirect_team = None
        self._indirect_first_robot = None

    def _track_indirect_contacts(self, robot_indices: frozenset[int]) -> None:
        if self._indirect_team is None:
            return
        if self._indirect_first_robot is None:
            takers = sorted(robot for robot in robot_indices if self.robot_teams.get(robot) == self._indirect_team)
            if not takers:
                return
            self._indirect_first_robot = takers[0]
            if len(robot_indices) == 1:
                return
        # Inputs contain contact onsets only, so renewed contact by the taker also counts.
        if robot_indices:
            self._event("Indirekte Torsperre aufgehoben: Zweite Ballberührung erkannt.")
            self._clear_indirect()

    def _event(self, message: str) -> None:
        if self._event_callback is not None:
            self._event_callback(message)

    def _reset_outside(self) -> None:
        self._clear_indirect()
        self.double_touch_rules.clear()
        self._pending_outside = None
        self._placement = None
        self._restart_at = None
        self._placement_failed = False
        self._previous_ball_position = None
        self.ball_position = None
        self._last_check_ball_outside = False

    def ballOutside(self) -> bool:  # noqa: N802 - retain the rule hook name
        """The ball is out only after its full sphere has cleared the outer marking edge."""
        return self.ball_position is not None and self.field.outside(self.ball_position)

    def _check_outside(self, game_state: MatchState) -> MatchState:
        assert self.simulation_time_ns is not None
        if self._pending_outside is not None:
            return self.handleBalloutside(game_state)
        if self._restart_at is not None:
            game_state = self._advance_restart(game_state)
            if game_state.state != "STATE_PLAYING":
                self._last_check_ball_outside = self.ballOutside()
                return game_state
        outside = self.ballOutside()
        if (
            outside
            and not self._last_check_ball_outside
            and self._previous_ball_position is not None
            and game_state.state == "STATE_PLAYING"
            and not game_state.stopped
        ):
            self._pending_outside = self.field.classify(
                self._previous_ball_position,
                self.ball_position,
                game_state,
                self.last_touch_team_id,
                self.simulation_time_ns,
            )
            if self._pending_outside is not None and self._pending_outside.kind == "GOAL" and self._indirect_team is not None:
                self._pending_outside = self.field.classify(
                    self._previous_ball_position, self.ball_position, game_state,
                    self.last_touch_team_id if self.last_touch_team_id is not None else self._indirect_team,
                    self.simulation_time_ns, allow_goal=False,
                )
                self._event("Kein Tor: Indirekte Spielfortsetzung ohne zweite Ballberührung.")
            if self._pending_outside is None:
                self._event("Ball im Aus: keine eindeutige Entscheidung ohne Grenzübertritt und Teamberührung.")
            else:
                self._restart_at = None
                decision = self._pending_outside
                self._event(f"Erkannt: {decision.kind}, Team {decision.team_id}; verzögerte Spielfortsetzung.")
        self._last_check_ball_outside = outside
        return game_state

    def handleBalloutside(self, game_state: MatchState) -> MatchState:  # noqa: N802
        """Apply the captured decision once, then wait for acknowledged ball placement."""
        decision = self._pending_outside
        assert self.simulation_time_ns is not None
        if decision is None or self.simulation_time_ns - decision.detected_at < OUTSIDE_DELAY_NS:
            return game_state
        if self._placement is None and not self._placement_failed:
            if game_state.state != "STATE_PLAYING" or game_state.stopped:
                self._pending_outside = None
                return game_state
            self._restart_is_goal = decision.kind == "GOAL"
            self._clear_indirect()
            self.double_touch_rules.clear()
            if self._restart_is_goal:
                teams = tuple(
                    replace(team, score=min(255, team.score + 1)) if team.team_number == decision.team_id else team
                    for team in game_state.teams
                )
                conceding = next(team.team_number for team in teams if team.team_number != decision.team_id)
                game_state = replace(
                    game_state, teams=teams, kicking_team=conceding, stopped=True,
                    set_play="SET_PLAY_NONE", secondary_time=0,
                )
            else:
                game_state = replace(
                    game_state, set_play=decision.kind, kicking_team=decision.team_id,
                    secondary_time=SET_PLAY_SECONDS, stopped=True,
                )
            self._clock_was_running = False
            self._event(f"Entscheidung: {decision.kind}, Team {decision.team_id}")
            try:
                self._placement = self.teleport_ball(*decision.position, 0.0)
            except Exception as error:
                self._placement_failed = True
                self._event(f"Ballplatzierung fehlgeschlagen; Spiel bleibt angehalten: {error}")
        if self._placement_failed or self._placement is None or not self._placement.done():
            return game_state
        try:
            result = self._placement.result()
            if not result.success:
                raise RuntimeError(result.message)
        except Exception as error:
            self._placement_failed = True
            self._event(f"Ballplatzierung fehlgeschlagen; Spiel bleibt angehalten: {error}")
            return game_state
        # Wait until observations have caught up with the service acknowledgement.
        if self.step_number < result.applied_step:
            return game_state
        self._placement = None
        self._pending_outside = None
        self._clear_indirect()
        if decision.kind != "GOAL":
            self.double_touch_rules.arm(decision.team_id)
        if decision.kind in ("SET_PLAY_INDIRECT_FREE_KICK", "SET_PLAY_THROW_IN"):
            self._indirect_team = decision.team_id
        self._restart_at = self.simulation_time_ns
        self.last_touch_team_id = None
        self._touching_ball = frozenset()
        self._last_check_ball_outside = self.ballOutside()
        updated = self._advance_restart(game_state)
        self._clock_was_running = updated.state == "STATE_PLAYING" and not updated.stopped
        self._event(f"Ball platziert: {updated.state}, stopped={updated.stopped}")
        return updated

    def _advance_restart(self, game_state: MatchState) -> MatchState:
        assert self.simulation_time_ns is not None and self._restart_at is not None
        elapsed = self.simulation_time_ns - self._restart_at
        if self._restart_is_goal:
            boundary = 0
            for phase, duration in STARTUP_PHASES:
                if phase == "STATE_INITIAL":
                    continue
                boundary += duration * NANOSECONDS_PER_SECOND
                if elapsed < boundary:
                    remaining = (boundary - elapsed + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND
                    return replace(game_state, state=phase, stopped=False, secondary_time=remaining)
        else:
            remaining = max(0, SET_PLAY_SECONDS - elapsed // NANOSECONDS_PER_SECOND)
            if remaining:
                return replace(game_state, secondary_time=remaining, stopped=False)
        self._restart_at = None
        if self._restart_is_goal:
            self._clock_was_running = False
        self._event("Spielfortsetzung: PLAYING")
        return replace(game_state, state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_NONE", secondary_time=0)

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
