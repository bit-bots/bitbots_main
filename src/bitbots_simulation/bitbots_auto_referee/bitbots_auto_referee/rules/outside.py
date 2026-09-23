"""Field geometry and decisions captured at the first complete ball exit."""

from dataclasses import dataclass, fields

from bitbots_auto_referee.config import RefereeConfig
from bitbots_auto_referee.core.observations import Position
from bitbots_auto_referee.core.state import MatchState

OUTSIDE_DELAY_NS = 2_000_000_000
SET_PLAY_SECONDS = 45


@dataclass(frozen=True)
class OutsideDecision:
    kind: str
    team_id: int
    position: tuple[float, float]
    detected_at: int


@dataclass(frozen=True)
class FieldGeometry:
    field_length: float = 9.0
    field_width: float = 6.0
    line_width: float = 0.05
    goal_width: float = 1.75
    goal_height: float = 1.2
    goal_area_length: float = 1.0
    goal_area_width: float = 3.0
    ball_radius: float = 0.07
    home_defends_negative_x: bool = False

    @classmethod
    def from_config(cls, config: RefereeConfig) -> "FieldGeometry":
        return cls(**{field.name: getattr(config, field.name) for field in fields(cls)})

    @property
    def boundaries(self) -> tuple[float, float]:
        margin = self.line_width / 2 + self.ball_radius
        return self.field_length / 2 + margin, self.field_width / 2 + margin

    def outside(self, position: Position) -> bool:
        x_limit, y_limit = self.boundaries
        return abs(position[0]) > x_limit or abs(position[1]) > y_limit

    def classify(
        self, previous: Position, current: Position, state: MatchState, last_touch: int | None, now_ns: int
    ) -> OutsideDecision | None:
        """Interpolate the first crossed boundary, including height at the goal opening."""
        crossings = []
        for axis, limit in enumerate(self.boundaries):
            delta = current[axis] - previous[axis]
            if abs(current[axis]) > limit and delta:
                sign = 1 if current[axis] > 0 else -1
                fraction = (sign * limit - previous[axis]) / delta
                if 0 <= fraction <= 1:
                    crossings.append((fraction, axis, sign))
        if not crossings:
            return None
        fraction, axis, sign = min(crossings)
        x, y, z = tuple(a + fraction * (b - a) for a, b in zip(previous, current, strict=True))
        home, away = (team.team_number for team in state.teams)
        home_negative = self.home_defends_negative_x == state.first_half
        defender = home if (sign < 0) == home_negative else away
        attacker = away if defender == home else home
        half_length, half_width = self.field_length / 2, self.field_width / 2
        if axis == 0 and abs(y) + self.ball_radius < self.goal_width / 2 and z + self.ball_radius < self.goal_height:
            return OutsideDecision("GOAL", attacker, (0.0, 0.0), now_ns)
        if last_touch not in (home, away):
            return None
        other = away if last_touch == home else home
        if axis == 1:
            return OutsideDecision(
                "SET_PLAY_THROW_IN", other,
                (max(-half_length, min(half_length, x)), sign * half_width), now_ns,
            )
        y_sign = 1 if y >= 0 else -1
        if last_touch == defender:
            return OutsideDecision(
                "SET_PLAY_CORNER_KICK", attacker, (sign * half_length, y_sign * half_width), now_ns,
            )
        return OutsideDecision(
            "SET_PLAY_GOAL_KICK", defender,
            (sign * (half_length - self.goal_area_length), y_sign * self.goal_area_width / 2), now_ns,
        )
