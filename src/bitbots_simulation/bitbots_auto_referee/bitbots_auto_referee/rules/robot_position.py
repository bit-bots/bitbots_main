"""Whole-body positioning rules using conservative horizontal collision bounds."""

import math

from bitbots_auto_referee.core.observations import RobotBounds, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.motion import GRACE_NS

LEAVING_MARGIN = 1.0
ESCAPE_SPEED = 0.03
INWARD_TOLERANCE = 0.01


def distance(bounds: RobotBounds, x: float, y: float) -> float:
    return math.hypot(max(bounds.min_x - x, x - bounds.max_x, 0.0), max(bounds.min_y - y, y - bounds.max_y, 0.0))


class RobotPositionRules:
    def __init__(self, robot_teams, robot_players, field, penalties, set_play_min_distance=None):
        self.robot_teams = robot_teams
        self.robot_players = robot_players
        self.field = field
        self.penalties = penalties
        self.set_play_min_distance = (
            2 * field.center_circle_radius if set_play_min_distance is None else set_play_min_distance
        )
        self._previous = None
        self._context = None
        self._restricted_since = {}
        self._kicker = None

    def configure_set_play_distance(self, distance: float) -> None:
        """Restart position grace periods when the required set-play distance changes."""
        if distance != self.set_play_min_distance:
            self.set_play_min_distance = distance
            self._restricted_since.clear()

    def _eligible(self, state, robot):
        return self.penalties.eligible(state, robot)

    def _own_negative(self, state, team_id):
        home_negative = self.field.home_defends_negative_x == state.first_half
        return home_negative if team_id == state.teams[0].team_number else not home_negative

    def _escaping(self, robot, observation):
        previous = self._previous
        if previous is None or robot not in previous.robot_positions or observation.ball_position is None:
            return False
        elapsed = (observation.time_ns - previous.time_ns) / 1e9
        if elapsed <= 0 or robot in previous.teleported_robots:
            return False
        x, y, _ = observation.robot_positions[robot]
        old_x, old_y, _ = previous.robot_positions[robot]
        vx, vy = (x - old_x) / elapsed, (y - old_y) / elapsed
        bx, by, _ = observation.ball_position
        radius = math.hypot(x - bx, y - by)
        if radius == 0:
            return False
        outward = (vx * (x - bx) + vy * (y - by)) / radius
        # Tangential progress around the ball is allowed; inward progress is not.
        return math.hypot(vx, vy) > ESCAPE_SPEED and outward >= -INWARD_TOLERANCE

    @staticmethod
    def _bounds(observation):
        return {
            robot: box
            for robot, box in observation.robot_bounds.items()
            if robot in observation.robot_positions
            and all(math.isfinite(value) for value in vars(box).values())
            and box.min_x <= box.max_x
            and box.min_y <= box.max_y
        }

    def _select_kicker(self, state, observation, bounds):
        circle = self.field.center_circle_radius + self.field.line_width / 2
        candidates = [
            robot
            for robot, box in bounds.items()
            if self.robot_teams.get(robot) == state.kicking_team
            and self._eligible(state, robot)
            and distance(box, 0, 0) <= circle
        ]
        if self._kicker in candidates:
            return self._kicker
        return (
            min(candidates, key=lambda robot: (math.hypot(*observation.robot_positions[robot][:2]), robot))
            if candidates
            else None
        )

    def _illegal_in_set(self, state, robot, box, kicker):
        half_x, half_y = self.field.field_length / 2, self.field.field_width / 2
        team_id = self.robot_teams[robot]
        own_half = box.max_x <= 0 if self._own_negative(state, team_id) else box.min_x >= 0
        on_field = box.min_x >= -half_x and box.max_x <= half_x and box.min_y >= -half_y and box.max_y <= half_y
        circle = self.field.center_circle_radius + self.field.line_width / 2
        if team_id == state.kicking_team and distance(box, 0, 0) <= circle:
            return robot != kicker or not on_field
        return not own_half or not on_field

    def legal_set_robots(self, state, observation) -> set[int] | None:
        """Require complete, legal SET positions for all configured unpenalized robots."""
        robots = {robot for robot in self.robot_teams if self._eligible(state, robot)}
        bounds = self._bounds(observation)
        if not robots or not robots <= bounds.keys() or observation.robot_positions.keys() - self.robot_teams.keys():
            return None
        kicker = self._select_kicker(state, observation, bounds)
        if any(self._illegal_in_set(state, robot, bounds[robot], kicker) for robot in robots):
            return None
        return robots

    def check(self, state: MatchState, observation: SimulationObservation) -> MatchState:
        context = (state.state, state.stopped, state.set_play, state.kicking_team, state.first_half)
        reset = self._previous is not None and (
            observation.time_ns < self._previous.time_ns or observation.step_number < self._previous.step_number
        )
        if reset or context != self._context or observation.ball_teleported:
            self._restricted_since.clear()
            self._kicker = None
        self._context = context
        if state.state == "STATE_INITIAL":
            self._restricted_since.clear()
            self._kicker = None
            self._previous = observation
            return state
        bounds = self._bounds(observation)
        self._kicker = self._select_kicker(state, observation, bounds)
        half_x, half_y = self.field.field_length / 2, self.field.field_width / 2
        for robot, box in sorted(bounds.items()):
            if not self._eligible(state, robot):
                self._restricted_since.pop(robot, None)
                continue
            if robot in observation.teleported_robots:
                self._restricted_since.pop(robot, None)
                continue
            outside_distance = math.hypot(
                max(box.min_x - half_x, -half_x - box.max_x, 0.0),
                max(box.min_y - half_y, -half_y - box.max_y, 0.0),
            )
            if outside_distance > LEAVING_MARGIN + self.field.line_width / 2:
                state = self.penalties.penalize(state, observation, robot, "PENALTY_LEAVING_THE_FIELD")
                self._restricted_since.pop(robot, None)
                continue
            team_id = self.robot_teams[robot]
            illegal = False
            if state.state == "STATE_SET":
                illegal = self._illegal_in_set(state, robot, box, self._kicker)
            elif (
                state.state == "STATE_PLAYING"
                and not state.stopped
                and state.set_play != "SET_PLAY_NONE"
                and team_id != state.kicking_team
            ):
                if observation.ball_position is not None:
                    bx, by, _ = observation.ball_position
                    illegal = distance(box, bx, by) < self.set_play_min_distance and not self._escaping(
                        robot, observation
                    )
                if state.set_play == "SET_PLAY_GOAL_KICK":
                    opponent_negative = not self._own_negative(state, team_id)
                    line = self.field.line_width / 2
                    area_min = -half_x - line if opponent_negative else half_x - self.field.penalty_area_length - line
                    area_max = -half_x + self.field.penalty_area_length + line if opponent_negative else half_x + line
                    overlaps = (
                        box.max_x >= area_min
                        and box.min_x <= area_max
                        and box.max_y >= -self.field.penalty_area_width / 2 - line
                        and box.min_y <= self.field.penalty_area_width / 2 + line
                    )
                    illegal = illegal or overlaps
            if illegal:
                since = self._restricted_since.setdefault(robot, observation.time_ns)
                if observation.time_ns - since >= GRACE_NS:
                    state = self.penalties.penalize(state, observation, robot, "PENALTY_ILLEGAL_POSITIONING")
                    self._restricted_since.pop(robot, None)
            else:
                self._restricted_since.pop(robot, None)
        self._restricted_since = {robot: since for robot, since in self._restricted_since.items() if robot in bounds}
        self._previous = observation
        return state
