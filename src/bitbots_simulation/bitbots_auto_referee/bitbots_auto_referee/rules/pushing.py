"""Contact-force pushing heuristics with explicit actor and legal-duel checks."""

import math
from dataclasses import dataclass, fields

from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState

SUSTAINED_NS = 5_000_000_000


@dataclass(frozen=True)
class PushingConfig:
    force_threshold: float = 20.0
    sustained_force_threshold: float = 2.0
    approach_speed: float = 0.03
    tilt_drop: float = 0.15
    angular_speed: float = 1.0
    effect_window: float = 0.5
    ball_center_tolerance: float = 0.25
    ball_reach: float = 0.7

    @classmethod
    def from_config(cls, config):
        return cls(**{field.name: getattr(config, f"pushing_{field.name}") for field in fields(cls)})


@dataclass(frozen=True)
class PushingFoul:
    actor: int
    victim: int
    position: tuple[float, float]


@dataclass
class PushEpisode:
    actor: int
    victim: int
    last_contact: int
    sustained_since: int | None
    upright: float | None
    height: float | None
    angular_speed: float | None
    peak_force: float
    strong_at: int | None
    position: tuple[float, float]


class PushingRules:
    def __init__(self, penalties, event, config: PushingConfig | None = None):
        self.penalties = penalties
        self.event = event
        self.config = config or PushingConfig()
        self.fouls: list[PushingFoul] = []
        self._previous = None
        self._episodes = {}
        self.contact_forces: dict[tuple[int, int], float] = {}

    def _eligible(self, state, robot):
        number = self.penalties.robot_players.get(robot)
        return any(
            team.team_number == self.penalties.robot_teams.get(robot)
            and number is not None and 1 <= number <= len(team.players)
            and team.players[number - 1].penalty == "PENALTY_NONE"
            for team in state.teams
        )

    def _ball_duel(self, pair, observation):
        if observation.ball_position is None:
            return False
        a, b = (observation.robot_positions[robot][:2] for robot in pair)
        ball = observation.ball_position[:2]
        length_squared = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        if length_squared == 0:
            return False
        projection = sum((ball[i] - a[i]) * (b[i] - a[i]) for i in (0, 1)) / length_squared
        midpoint = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        if not 0 <= projection <= 1 or math.dist(ball, midpoint) > self.config.ball_center_tolerance:
            return False
        for robot in pair:
            yaw = observation.robot_yaws.get(robot)
            motion = observation.robot_motion.get(robot)
            if yaw is None or not math.isfinite(yaw) or motion is None or motion.upright < 0.7:
                return False
            x, y, _ = observation.robot_positions[robot]
            dx, dy = ball[0] - x, ball[1] - y
            distance = math.hypot(dx, dy)
            if distance > self.config.ball_reach or dx * math.cos(yaw) + dy * math.sin(yaw) < distance * 0.5:
                return False
        return True

    def check(self, state: MatchState, observation: SimulationObservation) -> MatchState:
        self.fouls = []
        previous = self._previous
        reset = previous is not None and (
            observation.time_ns < previous.time_ns or observation.step_number < previous.step_number
        )
        if reset:
            self._episodes.clear()
            previous = None
        self._previous = observation
        self.contact_forces = {}
        active = set()
        now = observation.time_ns
        for contact in observation.robot_contacts:
            pair = (contact.robot_a, contact.robot_b)
            if pair[0] >= pair[1] or not all(
                math.isfinite(value) and value >= 0 for value in (contact.force, contact.approach_a, contact.approach_b)
            ):
                continue
            self.contact_forces[pair] = contact.force
            if any(robot not in observation.robot_positions or robot in observation.teleported_robots
                   or not self._eligible(state, robot) for robot in pair):
                self._episodes.pop(pair, None)
                continue
            approach = [contact.approach_a, contact.approach_b]
            if previous is not None and now > previous.time_ns and all(robot in previous.robot_positions for robot in pair):
                a, b = (observation.robot_positions[robot][:2] for robot in pair)
                length = math.dist(a, b)
                if length > 0 and not any(robot in previous.teleported_robots for robot in pair):
                    direction = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
                    dt = (now - previous.time_ns) / 1e9
                    for index, robot in enumerate(pair):
                        displacement = [observation.robot_positions[robot][i] - previous.robot_positions[robot][i] for i in (0, 1)]
                        speed = sum(displacement[i] * direction[i] for i in (0, 1)) / dt
                        approach[index] = max(approach[index], speed if index == 0 else -speed)
            moving = [speed >= self.config.approach_speed for speed in approach]
            if all(moving) or self._ball_duel(pair, observation):
                self._episodes.pop(pair, None)
                continue
            episode = self._episodes.get(pair)
            actor = pair[moving.index(True)] if any(moving) else (episode.actor if episode is not None else None)
            if actor is None or contact.force < self.config.sustained_force_threshold:
                if episode is not None:
                    episode.sustained_since = None
                continue
            victim = pair[1] if actor == pair[0] else pair[0]
            if contact.position is not None and all(math.isfinite(value) for value in contact.position):
                position = contact.position[:2]
            else:
                position = tuple(
                    (observation.robot_positions[actor][axis] + observation.robot_positions[victim][axis]) / 2
                    for axis in (0, 1)
                )
            if episode is None or episode.actor != actor:
                baseline = previous.robot_motion.get(victim) if previous is not None else None
                episode = PushEpisode(
                    actor, victim, now, now,
                    baseline.upright if baseline else None,
                    baseline.relative_height if baseline else None,
                    baseline.angular_speed if baseline else None,
                    contact.force, None, position,
                )
                self._episodes[pair] = episode
            if episode.sustained_since is None:
                episode.sustained_since = now
            episode.last_contact = now
            episode.position = position
            episode.peak_force = max(episode.peak_force, contact.force)
            if contact.force >= self.config.force_threshold:
                if episode.strong_at is None or now - episode.strong_at > self.config.effect_window * 1e9:
                    baseline = previous.robot_motion.get(victim) if previous is not None else None
                    if baseline is not None:
                        episode.upright = baseline.upright
                        episode.height = baseline.relative_height
                        episode.angular_speed = baseline.angular_speed
                episode.strong_at = now
            active.add(pair)
        for pair, episode in list(self._episodes.items()):
            if (
                any(robot not in observation.robot_positions or robot in observation.teleported_robots
                    or not self._eligible(state, robot) for robot in pair)
                or self._ball_duel(pair, observation)
            ):
                del self._episodes[pair]
                continue
            if pair not in active:
                episode.sustained_since = None
            motion = observation.robot_motion.get(episode.victim)
            recent_strong = episode.strong_at is not None and now - episode.strong_at <= self.config.effect_window * 1e9
            destabilized = (
                recent_strong and motion is not None and episode.upright is not None and episode.upright >= 0.7
                and (
                    episode.upright - motion.upright >= self.config.tilt_drop
                    or (episode.height >= 0.7 and motion.relative_height < 0.5)
                    or motion.angular_speed - episode.angular_speed >= self.config.angular_speed
                )
            )
            sustained = episode.sustained_since is not None and now - episode.sustained_since >= SUSTAINED_NS
            if destabilized or sustained:
                self.event(
                    f"Pushing: Roboter {episode.actor} gegen {episode.victim}; "
                    f"Spitzenkraft {episode.peak_force:.2f} N; "
                    f"{'Destabilisierung' if destabilized else 'anhaltender Kontakt'}."
                )
                updated = self.penalties.penalize(state, observation, episode.actor, "PENALTY_PUSHING")
                if updated != state:
                    self.fouls.append(PushingFoul(episode.actor, episode.victim, episode.position))
                state = updated
                del self._episodes[pair]
            elif pair not in active and now - episode.last_contact > self.config.effect_window * 1e9:
                del self._episodes[pair]
        return state
