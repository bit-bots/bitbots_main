"""Restart double touches based on distinct significant contact episodes."""

import math
from dataclasses import dataclass, fields

from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState

MIN_ACTIVE_PLAYERS = 3


@dataclass(frozen=True)
class DoubleTouchConfig:
    min_force: float = 2.0
    min_impulse: float = 0.02
    release_time: float = 0.05

    @classmethod
    def from_config(cls, config):
        return cls(**{field.name: getattr(config, f"double_touch_{field.name}") for field in fields(cls)})


@dataclass
class ContactEpisode:
    impulse: float = 0.0
    peak_force: float = 0.0
    counted: bool = False
    released_at: int | None = None


class DoubleTouchRules:
    def __init__(self, robot_teams, robot_players, field, event, config=None):
        self.robot_teams = robot_teams
        self.robot_players = robot_players
        self.field = field
        self.event = event
        self.config = config or DoubleTouchConfig()
        self.team_id = None
        self.first_robot = None
        self._contacts = {}
        self._suppressed = set()
        self._previous_time = None
        self._previous_step = None
        self._previous_phase = None
        self.active_players: dict[int, int] = {}

    def clear(self):
        self.team_id = None
        self.first_robot = None
        self._contacts.clear()
        self._suppressed.clear()

    def arm(self, team_id):
        self.clear()
        self.team_id = team_id

    def _eligible(self, state, robot):
        number = self.robot_players.get(robot)
        return any(
            team.team_number == self.robot_teams.get(robot)
            and number is not None and 1 <= number <= len(team.players)
            and team.players[number - 1].penalty == "PENALTY_NONE"
            for team in state.teams
        )

    def count_active_players(self, state, observation):
        result = {team.team_number: set() for team in state.teams}
        half_x, half_y = self.field.field_length / 2, self.field.field_width / 2
        for robot, position in observation.robot_positions.items():
            if not self._eligible(state, robot) or not all(math.isfinite(value) for value in position):
                continue
            box = observation.robot_bounds.get(robot)
            if box is None:
                on_field = -half_x < position[0] < half_x and -half_y < position[1] < half_y
            else:
                on_field = (
                    all(math.isfinite(value) for value in vars(box).values())
                    and -half_x <= box.min_x <= box.max_x <= half_x
                    and -half_y <= box.min_y <= box.max_y <= half_y
                )
            if on_field:
                result[self.robot_teams[robot]].add(self.robot_players[robot])
        return {team: len(players) for team, players in result.items()}

    def check(self, state: MatchState, observation: SimulationObservation, enabled: bool = True) -> int | None:
        """Return the fouling team, leaving the caller to award the opponent's free kick."""
        now = observation.time_ns
        reset = self._previous_time is not None and (
            now < self._previous_time or observation.step_number < self._previous_step
        )
        dt = 0.0 if self._previous_time is None or reset else (now - self._previous_time) / 1e9
        self._previous_time, self._previous_step = now, observation.step_number
        if reset:
            self.clear()
            self._previous_phase = None
        if self._previous_phase == "STATE_SET" and state.state == "STATE_PLAYING" and state.set_play == "SET_PLAY_NONE":
            if state.kicking_team in {team.team_number for team in state.teams}:
                self.arm(state.kicking_team)
            else:
                self.clear()
        self._previous_phase = state.state
        self.active_players = self.count_active_players(state, observation)
        if state.state != "STATE_PLAYING":
            self.clear()
        touching = observation.touching_ball.intersection(observation.robot_positions)
        self._suppressed.intersection_update(touching)
        self._suppressed.update(touching.intersection(observation.teleported_robots))
        if not enabled or state.stopped or self.team_id is None or observation.ball_position is None:
            self._contacts.clear()
            self._suppressed.update(touching)
            return None
        if observation.ball_teleported:
            self._contacts.clear()
            self._suppressed.update(touching)
            return None
        for robot in list(self._contacts):
            if robot in observation.teleported_robots or robot not in observation.robot_positions:
                del self._contacts[robot]
                continue
            if robot not in touching:
                episode = self._contacts[robot]
                if episode.released_at is None:
                    episode.released_at = now
                if now - episode.released_at >= self.config.release_time * 1e9:
                    del self._contacts[robot]
        significant = set()
        for robot in touching - self._suppressed:
            force = observation.ball_contact_forces.get(robot, 0.0)
            if not math.isfinite(force) or force < 0:
                continue
            episode = self._contacts.get(robot)
            if episode is not None and episode.released_at is not None:
                if now - episode.released_at >= self.config.release_time * 1e9:
                    episode = None
            if episode is None:
                episode = ContactEpisode()
                self._contacts[robot] = episode
            episode.released_at = None
            episode.peak_force = max(episode.peak_force, force)
            episode.impulse += force * dt
            if not episode.counted and episode.peak_force >= self.config.min_force and episode.impulse >= self.config.min_impulse:
                episode.counted = True
                significant.add(robot)
        if not significant:
            return None
        if self.first_robot is None:
            takers = {robot for robot in significant if self.robot_teams.get(robot) == self.team_id and self._eligible(state, robot)}
            if len(significant) != 1 or len(takers) != 1:
                self.clear()
                return None
            self.first_robot = next(iter(takers))
            return None
        if any(robot != self.first_robot for robot in significant):
            self.clear()
            return None
        team = self.team_id
        if self._eligible(state, self.first_robot) and self.active_players.get(team, 0) >= MIN_ACTIVE_PLAYERS:
            self.event(f"Double Touch: Roboter {self.first_robot}, Team {team}, {self.active_players[team]} aktive Spieler.")
            self.clear()
            return team
        return None
